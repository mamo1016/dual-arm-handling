"""
joint_state_translator.py — bridges a joint-solution topic (Float64MultiArray)
to /joint_states (JointState) so robot_state_publisher and rviz2 can display
the arm.

The solution message carries either:
  * 7 values  — a single joint setpoint; the arm eases from its current pose to
    it over anim_duration, or
  * 7*K values — a pre-computed joint TRAJECTORY of K waypoints (used for the
    straight-line Cartesian moves); the arm plays through the waypoints over
    anim_duration so the gripper follows the exact path the planner generated.

Run:
    ros2 run dual_arm joint_state_translator
"""
import numpy as np
import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from std_msgs.msg import Float64MultiArray  # pyright: ignore[reportMissingImports]
from sensor_msgs.msg import JointState  # pyright: ignore[reportMissingImports]

ARM_JOINT_SUFFIXES = [
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7',
]
GRIPPER_JOINT_SUFFIXES = ['finger_joint1', 'finger_joint2']
GRIPPER_DEFAULT     = [0.0, 0.0]   # closed
PUB_HZ        = 20.0          # publish rate (Hz)
ANIM_DURATION = 2.0           # default seconds to move between targets
PUB_DT        = 1.0 / PUB_HZ


class JointStateTranslator(Node):
    def __init__(self):
        super().__init__('joint_state_translator')

        # joint_prefix selects which arm's joints this translator drives, e.g.
        # 'openarm_' (single arm) or 'arm_a_openarm_'/'arm_b_openarm_' in the
        # dual-arm scene. Both translators publish to the SAME global
        # /joint_states; robot_state_publisher merges the partial messages by
        # name, holding the other arm at its last value.
        prefix = self.declare_parameter('joint_prefix', 'openarm_').value
        self._arm_joint_names = [prefix + s for s in ARM_JOINT_SUFFIXES]
        self._gripper_joint_names = [prefix + s for s in GRIPPER_JOINT_SUFFIXES]
        solution_topic = self.declare_parameter(
            'solution_topic', '/joint_solution').value
        # Seconds to ease between targets. Short values make snappy motion for
        # fast demos (e.g. the hot-potato handoff); the default stays smooth.
        self._anim_duration = self.declare_parameter(
            'anim_duration', ANIM_DURATION).value

        self._current_q = np.zeros(7)
        # Waypoint path the arm is currently following (rows = 7-DOF configs).
        # Starts as a single point (idle, holds zero pose).
        self._waypoints = np.zeros((1, 7))
        self._t_anim    = self._anim_duration   # >= duration means "idle"

        self.create_subscription(
            Float64MultiArray, solution_topic, self._on_target, 10)
        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(PUB_DT, self._publish)

        self.get_logger().info(
            f'Joint state translator ready — joints {self._arm_joint_names[0]}…'
            f' from {solution_topic}')

    def _on_target(self, msg: Float64MultiArray):
        data = np.array(msg.data)
        k = len(data) // 7
        configs = data[:k * 7].reshape(k, 7)
        # Always start the path from where the arm actually is now, so playback
        # is continuous even if a message was dropped. A single setpoint becomes
        # a 2-point path (ease there); a trajectory is played through in order.
        path = np.vstack([self._current_q, configs])
        # Shortest angular route per joint. IK can return a setpoint a full turn
        # away from an identical-looking pose (e.g. +5.6 rad where -0.68 rad has
        # the SAME geometry), which makes the arm sweep ~360° for nothing.
        # np.unwrap removes per-joint jumps > pi, anchored at the current pose
        # (row 0 is left untouched), so each joint takes the minimal path to a
        # kinematically identical target. It's a no-op for an already-smooth
        # path, so the resolved-rate trajectories are unaffected.
        self._waypoints = np.unwrap(path, axis=0)
        self._t_anim = 0.0
        self.get_logger().info(
            f'New {"trajectory (%d pts)" % k if k > 1 else "setpoint"} '
            f'— animating over {self._anim_duration}s')

    def _publish(self):
        # Ease along the waypoint path: smoothstep on the overall progress u,
        # linear between adjacent waypoints. For a 2-point path this is a simple
        # smoothstep lerp; for a dense trajectory it tracks the planned path.
        if self._t_anim < self._anim_duration:
            self._t_anim += PUB_DT
            u = min(self._t_anim / self._anim_duration, 1.0)
            s = u * u * (3 - 2 * u)              # smoothstep 3u²-2u³
            wp = self._waypoints
            f = s * (len(wp) - 1)                # fractional waypoint index
            i = int(f)
            if i >= len(wp) - 1:
                self._current_q = wp[-1]
            else:
                self._current_q = wp[i] + (f - i) * (wp[i + 1] - wp[i])

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._arm_joint_names + self._gripper_joint_names
        msg.position = self._current_q.tolist() + GRIPPER_DEFAULT
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
