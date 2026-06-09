"""
arm_planner.py — receives a Cartesian target and computes joint angles via IK.

Subscribes to a target topic (geometry_msgs/Pose), runs inverse kinematics for
the 7-DOF OpenArm, and publishes the joint solution (std_msgs/Float64MultiArray)
for a joint_state_translator to animate.

Run:
    ros2 run dual_arm arm_planner
"""
import numpy as np
import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import Pose  # pyright: ignore[reportMissingImports]
from std_msgs.msg import Float64MultiArray  # pyright: ignore[reportMissingImports]
from ament_index_python.packages import get_package_share_directory  # pyright: ignore[reportMissingImports]
from spatialmath import SE3, UnitQuaternion
from roboticstoolbox import p_servo

from dual_arm.openarm import OpenArm

# A solution is accepted if its achieved pose is within these tolerances,
# regardless of ikine_LM's `success` flag (which has false negatives).
POS_TOL_MM  = 2.0
ORI_TOL_DEG = 2.0


class ArmPlanner(Node):
    def __init__(self):
        super().__init__('arm_planner')
        # The URDF is installed to the package share directory; load it from
        # there so this works under both colcon build and --symlink-install.
        share_dir = get_package_share_directory('dual_arm')
        self._robot = OpenArm(urdf_dir=share_dir)

        # robot_state_publisher renders each arm's link0 at a fixed world pose
        # (the world_to_*base joint in the dual URDF). We keep the IK model's
        # base at IDENTITY and instead transform incoming WORLD targets into the
        # arm's base frame before solving — this reuses the exact, proven
        # single-arm IK path.
        #
        # (Setting robot.base to a *rotated* offset instead is mathematically
        # equivalent but makes ikine_LM's masked position solve converge to the
        # wrong configuration — verified empirically. So we transform the target
        # rather than rotate the model.)
        #
        # base_xyz / base_rpy default to identity (link0 at the world origin).
        # For a second arm whose base is offset/rotated, pass that pose; world
        # targets are then mapped through base.inv() into link0's frame.
        base_xyz = self.declare_parameter('base_xyz', [0.0, 0.0, 0.0]).value
        base_rpy = self.declare_parameter('base_rpy', [0.0, 0.0, 0.0]).value
        self._robot.base = SE3()
        self._world_to_base = (
            SE3(base_xyz[0], base_xyz[1], base_xyz[2]) *
            SE3.RPY(base_rpy, order='zyx')).inv()

        target_topic = self.declare_parameter(
            'target_topic', '/target').value
        solution_topic = self.declare_parameter(
            'solution_topic', '/joint_solution').value
        # best_effort: when an oriented target is out of reach, move to the
        # NEAREST reachable pose (closest position, best orientation) instead of
        # refusing to move. Off by default (strict).
        self._best_effort = self.declare_parameter('best_effort', False).value

        self._last_q = self._robot.qr[:7].copy()   # seed for next IK call

        # Pick the TCP end-effector explicitly (don't rely on ee_links[0] ordering)
        ee_names = [ln.name for ln in self._robot.ee_links]
        if 'openarm_hand_tcp' in ee_names:
            self._ee = self._robot.ee_links[ee_names.index('openarm_hand_tcp')]
        else:
            self._ee = self._robot.ee_links[0]
        self.get_logger().info(f'Using end-effector: {self._ee.name}  (all EE links: {ee_names})')

        self.create_subscription(Pose, target_topic, self._on_target, 10)
        self._joint_pub = self.create_publisher(Float64MultiArray, solution_topic, 10)
        self.get_logger().info(
            f'Arm planner ready — waiting for targets on {target_topic}')

    def _on_target(self, msg: Pose):
        world_xyz = (msg.position.x, msg.position.y, msg.position.z)
        q = msg.orientation

        # A zero quaternion (the Pose default we override) is the sentinel for
        # "position only, orientation free". A unit quaternion requests a
        # specific gripper orientation.
        quat_norm = (q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2) ** 0.5
        constrain = abs(quat_norm - 1.0) < 1e-3

        if constrain:
            # Full world pose (position + desired rotation), mapped into the
            # arm's base frame. self._world_to_base is an SE3, so this composes
            # both translation AND rotation correctly.
            T_world = SE3(*world_xyz) * UnitQuaternion(q.w, [q.x, q.y, q.z]).SE3()
            T_base = self._world_to_base * T_world
            mask = [1, 1, 1, 1, 1, 1]
        else:
            # Position-only — orientation left to the solver.
            T_base = self._world_to_base * SE3(*world_xyz)
            mask = [1, 1, 1, 0, 0, 0]

        self.get_logger().info(
            f'Received target (world): {tuple(round(v, 3) for v in world_xyz)} '
            f'| {"6-DOF" if constrain else "pos-only"}')

        # For an orientation-constrained move, stream a straight-line Cartesian
        # TRAJECTORY rather than a single joint setpoint. Independent IK at the
        # endpoints lands on different branches of this redundant arm, so
        # joint-space interpolation between them swings the hand far off the
        # line. Resolved-rate servoing keeps the gripper on the straight path at
        # constant orientation.
        if constrain:
            traj = self._resolved_rate_traj(self._last_q, T_base)
            if traj is not None:
                pos_mm, ori_deg = self._pose_error(traj[-1], T_base, True)
                self.get_logger().info(
                    f'IK trajectory → {len(traj)} pts | '
                    f'pos error: {pos_mm:.1f} mm | ori error: {ori_deg:.1f}°')
                self._last_q = traj[-1]
                flat = np.asarray(traj, dtype=float).reshape(-1).tolist()
                self._joint_pub.publish(Float64MultiArray(data=flat))
                return
            # Servo failed to converge — fall back to single-shot IK below.
            self.get_logger().warn('Resolved-rate servo did not converge; '
                                   'falling back to single-shot IK')

        # Single-shot IK, judging the result by the ACHIEVED pose error rather
        # than the ikine_LM `success` flag — the LM solver sometimes flags a
        # perfectly good solution (0 mm / 0°) as failed when it stops on its
        # iteration limit. We seed from the last solution (smooth motion), and
        # on a poor result retry from the rest config, keeping whichever is best.
        best_q, best_pos, best_ori = None, None, None
        for seed in (self._last_q, self._robot.qr[:7]):
            sol = self._robot.ikine_LM(
                T_base, q0=seed, end=self._ee, mask=mask, ilimit=300)
            pos_mm, ori_deg = self._pose_error(sol.q[:7], T_base, constrain)
            if best_q is None or (pos_mm + (ori_deg or 0)) < (best_pos + (best_ori or 0)):
                best_q, best_pos, best_ori = sol.q[:7], pos_mm, ori_deg
            if pos_mm <= POS_TOL_MM and (not constrain or ori_deg <= ORI_TOL_DEG):
                break   # good enough — no need to try more seeds

        ori_msg = f' | ori error: {best_ori:.1f}°' if constrain else ''
        if best_pos <= POS_TOL_MM and (not constrain or best_ori <= ORI_TOL_DEG):
            self.get_logger().info(
                f'IK solved → pos error: {best_pos:.1f} mm{ori_msg}')
            self._last_q = best_q   # seed next call with this solution
            self._joint_pub.publish(Float64MultiArray(data=best_q.tolist()))
        elif self._best_effort:
            self.get_logger().warn(
                f'IK best-effort for world target {world_xyz} '
                f'(closest: {best_pos:.0f} mm{ori_msg}) — moving to nearest reachable pose')
            self._last_q = best_q
            self._joint_pub.publish(Float64MultiArray(data=best_q.tolist()))
        else:
            self.get_logger().error(
                f'IK failed for world target {world_xyz} '
                f'(best: {best_pos:.0f} mm{ori_msg}) — may be out of reach')

    def _pose_error(self, q, T_base, constrain):
        """Achieved (position mm, orientation deg) error of joint solution q."""
        T_ach = self._robot.fkine(q, end=self._ee)
        pos_mm = np.linalg.norm(T_ach.t - T_base.t) * 1000
        ori_deg = None
        if constrain:
            R_err = T_ach.R.T @ T_base.R
            ori_deg = np.degrees(np.arccos(
                np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0)))
        return pos_mm, ori_deg

    def _resolved_rate_traj(self, q_start, T_goal,
                            gain=1.5, dt=0.05, max_iter=400):
        """Resolved-rate servo from q_start to the goal pose, returning the
        joint trajectory (list of 7-vectors) that drives the TCP there along a
        straight Cartesian line at constant orientation.

        Uses the BODY-frame Jacobian (jacobe) to match p_servo's body-frame
        velocity twist, with damped least squares for stability near
        singularities. Returns None if it fails to converge in tolerance.
        """
        q = np.concatenate([np.asarray(q_start, float), [0.0, 0.0]])  # pad grip
        traj = [q[:7].copy()]
        arrived = False
        for _ in range(max_iter):
            Te = self._robot.fkine(q, end=self._ee)
            v, arrived = p_servo(Te, T_goal, gain=gain, threshold=0.002)
            J = self._robot.jacobe(q, end=self._ee)          # 6x7, body frame
            dq = J.T @ np.linalg.solve(J @ J.T + 0.0025 * np.eye(6), v)  # DLS
            q = q.copy()
            q[:7] = q[:7] + dq * dt
            traj.append(q[:7].copy())
            if arrived:
                break
        if not arrived:
            return None
        pos_mm, ori_deg = self._pose_error(traj[-1], T_goal, True)
        if pos_mm > POS_TOL_MM or ori_deg > ORI_TOL_DEG:
            return None
        # Resample to a fixed number of waypoints spaced UNIFORMLY along the TCP
        # arc length. The servo decelerates near the goal (gain-based), so its
        # raw waypoints bunch up at the end; uniform arc-length spacing makes the
        # translator's progress map linearly to Cartesian distance.
        return self._resample_arclength(traj, n=40)

    def _resample_arclength(self, traj, n=40):
        pts = np.array([self._robot.fkine(q, end=self._ee).t for q in traj])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = s[-1]
        if total < 1e-6:                      # negligible motion
            return [traj[0], traj[-1]]
        out = []
        for st in np.linspace(0.0, total, n):
            j = int(np.searchsorted(s, st))
            j = max(1, min(j, len(traj) - 1))
            denom = s[j] - s[j - 1]
            f = (st - s[j - 1]) / denom if denom > 1e-9 else 0.0
            out.append(traj[j - 1] + f * (traj[j] - traj[j - 1]))
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ArmPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
