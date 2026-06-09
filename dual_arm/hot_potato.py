"""
hot_potato.py — two arms play "HOT POTATO": they fling a single glowing cube
back and forth across the workspace, faster and faster, as if it were too hot to
hold. A light-hearted showcase of the dual-arm handoff pipeline.

Pipeline (per arm):
    hot_potato ──/arm_X/target (Pose)──▶ arm_planner_X ──▶
        /arm_X/joint_solution ──▶ joint_state_translator_X ──▶
        /joint_states ──▶ robot_state_publisher ──TF──▶ rviz2

The "potato" is a marker that tracks whichever arm currently HOLDS it (that
arm's TCP frame). Each beat the two hands meet at a shared CENTER point and the
holder switches, then the new holder yanks the cube back to its own side — so it
looks tossed, caught, and snatched away, all with no physics engine.

The beat period ramps DOWN over several passes (faster and faster), then resets
to slow — the comedic "okay okay, slow down… GO!" loop.

Run:
    ros2 launch dual_arm hot_potato.launch.py
"""
import math

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from rclpy.time import Time as RclpyTime  # pyright: ignore[reportMissingImports]
from rclpy.duration import Duration  # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import Pose  # pyright: ignore[reportMissingImports]
from visualization_msgs.msg import Marker, MarkerArray  # pyright: ignore[reportMissingImports]
from std_msgs.msg import ColorRGBA  # pyright: ignore[reportMissingImports]
from tf2_ros.buffer import Buffer  # pyright: ignore[reportMissingImports]
from tf2_ros.transform_listener import TransformListener  # pyright: ignore[reportMissingImports]


# --- Scene geometry, all in the shared `world` frame -----------------------
# Arm bases (see the dual URDF): arm_a at the world origin, arm_b at (-0.8,0,0)
# rotated 180°, facing each other across the workspace. The CENTER handoff point
# sits on the shared midline so BOTH arms reach it; each HOME is tucked near
# that arm's own base. Targets are position-only (orientation free) so they stay
# easily reachable and the clip never stalls on a failed IK solve.
CENTER = (-0.40, 0.12, 0.45)   # shared handoff point — cube arcs UP here (the "toss")
A_HOME = (-0.18, 0.20, 0.38)   # arm_a's side (near its base at x=0)
B_HOME = (-0.62, 0.20, 0.38)   # arm_b's side (near its base at x=-0.8)
CUBE_SIZE = (0.07, 0.07, 0.07)

# Four-beat loop. The cube follows the HOLDER's gripper the whole time:
#   a_present — arm_a brings the cube to CENTER, arm_b reaches in   (holder a)
#   b_recoil  — arm_b has it now, yanks back to B_HOME; arm_a recoils (holder b)
#   b_present — arm_b brings it to CENTER, arm_a reaches in          (holder b)
#   a_recoil  — arm_a has it now, yanks back to A_HOME; arm_b recoils (holder a)
# The holder switches at each present→recoil boundary, where both TCPs are
# together at CENTER, so the cube changes hands invisibly.
PHASES  = ['a_present', 'b_recoil', 'b_present', 'a_recoil']
HOLDER  = {'a_present': 'a', 'b_recoil': 'b', 'b_present': 'b', 'a_recoil': 'a'}
TARGETS = {
    'a_present': (CENTER, CENTER),
    'b_recoil':  (A_HOME, B_HOME),
    'b_present': (CENTER, CENTER),
    'a_recoil':  (A_HOME, B_HOME),
}

# --- Tempo -----------------------------------------------------------------
TICK_HZ      = 50.0          # coordinator tick (the beat timer accumulates on this)
SCENE_HZ     = 20.0          # marker publish rate — cube must look smooth
PERIOD_START = 1.10          # seconds per beat at the start (slow, deliberate)
PERIOD_MIN   = 0.55          # fastest beat (frantic)
PERIOD_DECAY = 0.86          # multiply the period each beat → speeds up
RESET_AFTER  = 9             # beats before resetting to slow (restart the gag)


class HotPotato(Node):
    def __init__(self):
        super().__init__('hot_potato')

        self._pub_a = self.create_publisher(Pose, '/arm_a/target', 10)
        self._pub_b = self.create_publisher(Pose, '/arm_b/target', 10)
        self._scene_pub = self.create_publisher(MarkerArray, '/scene/markers', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._phase_idx = -1            # first beat advances to 0 ('a_present')
        self._holder = 'a'              # arm_a starts holding the cube
        self._last_cube = A_HOME        # fallback before the first TF arrives
        self._period = PERIOD_START
        self._accum = 0.0
        self._beats = 0

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(1.0 / SCENE_HZ, self._publish_scene)
        self.get_logger().info('🔥 Hot potato! Two arms, one very hot cube. Started.')

    # --- Variable-tempo beat clock -----------------------------------------
    def _tick(self):
        """Accumulate time and fire a beat once the (shrinking) period elapses."""
        self._accum += 1.0 / TICK_HZ
        if self._accum >= self._period:
            self._accum = 0.0
            self._beat()

    def _beat(self):
        self._phase_idx = (self._phase_idx + 1) % len(PHASES)
        phase = PHASES[self._phase_idx]
        self._holder = HOLDER[phase]
        target_a, target_b = TARGETS[phase]
        self._publish_target(self._pub_a, target_a)
        self._publish_target(self._pub_b, target_b)

        self.get_logger().info(
            f'beat {self._beats:>2} | {phase:<10} | holder=arm_{self._holder} '
            f'| period={self._period:.2f}s')

        self._beats += 1
        if self._beats % RESET_AFTER == 0:
            self._period = PERIOD_START          # phew — slow down… and GO again
        else:
            self._period = max(PERIOD_MIN, self._period * PERIOD_DECAY)

    def _publish_target(self, pub, xyz):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = xyz
        # geometry_msgs/Quaternion defaults to w=1 (a valid orientation); zero it
        # so the planner treats this as a POSITION-ONLY target (orientation free).
        msg.orientation.w = 0.0
        pub.publish(msg)

    # --- Scene: just the one glowing potato --------------------------------
    def _publish_scene(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        cx, cy, cz = self._cube_pos()
        self._last_cube = (cx, cy, cz)
        arr.markers.append(self._make_cube((cx, cy, cz), now))

        self._scene_pub.publish(arr)

    def _cube_pos(self):
        """The cube rides the current holder's gripper TCP. On a momentary TF
        miss, hold the last known spot rather than snapping (avoids strobing)."""
        frame = f'arm_{self._holder}_openarm_hand_tcp'
        try:
            tf = self._tf_buffer.lookup_transform(
                'world', frame, RclpyTime(), timeout=Duration(seconds=0.05))
            return (tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z)
        except Exception:
            return self._last_cube

    def _make_cube(self, pos, stamp):
        # Flicker between orange and red so it reads as "glowing hot".
        t = self.get_clock().now().nanoseconds * 1e-9
        pulse = 0.5 + 0.5 * math.sin(t * 8.0)
        color = ColorRGBA(r=1.0, g=0.12 + 0.38 * pulse, b=0.0, a=1.0)
        return self._make_box(0, 'potato', pos, CUBE_SIZE, color, stamp)

    def _make_box(self, mid, ns, pos, size, color, stamp):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = size
        m.color = color
        return m


def main(args=None):
    rclpy.init(args=args)
    node = HotPotato()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
