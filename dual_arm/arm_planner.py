"""
arm_planner.py — receives a Cartesian target and computes joint angles via IK.

Subscribes to a target topic (geometry_msgs/Pose), runs inverse kinematics for
the 7-DOF OpenArm, and publishes the joint solution (std_msgs/Float64MultiArray)
for a joint_state_translator to animate.

An optional workspace KEEP-OUT volume (e.g. the relay demo's table, given by the
``keepout_box`` parameter) constrains the WHOLE ARM, not just the TCP: IK
solutions whose links would sweep through the volume are rejected and re-solved
from other seeds, and the resolved-rate servo steers offending links away
through the null space of the end-effector task.

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

# Keep-out constraint tuning (active only when the keepout_box param is set).
KEEPOUT_SAMPLES  = 5      # points sampled along each link segment
KEEPOUT_ACTIVATE = 0.02   # m before the margin at which null-space steering starts
KEEPOUT_GAIN     = 4.0    # null-space repulsion gain (m/s per m of penetration)
KEEPOUT_IK_SEEDS = 8      # max IK seeds to try when hunting a clearance-safe branch


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
        self._base_pose = (
            SE3(base_xyz[0], base_xyz[1], base_xyz[2]) *
            SE3.RPY(base_rpy, order='zyx'))
        self._world_to_base = self._base_pose.inv()

        # --- Optional workspace keep-out (e.g. the relay demo's table) ------
        # keepout_box = [xmin, xmax, ymin, ymax, z_top] in the WORLD frame. The
        # volume below z_top inside the XY footprint is treated as solid (like
        # a real table): no sampled point of any link may enter it. Fewer than
        # 5 values (the default) disables the constraint.
        box = self.declare_parameter('keepout_box', [0.0]).value
        self._keepout = np.asarray(box, float) if len(box) == 5 else None
        # Arm links keep this much clearance above the slab; the hand/finger
        # links use a smaller margin so the gripper can still descend to a
        # parcel resting ON the table.
        clear_arm = self.declare_parameter('keepout_clearance', 0.05).value
        clear_hand = self.declare_parameter('keepout_hand_clearance', 0.005).value
        # Per-segment sampling table: (parent frame, own frame, margin, link).
        # fkine_all returns [base frame] + one frame per link, hence the +1.
        idx = {ln.name: i for i, ln in enumerate(self._robot.links)}
        self._segments = [
            (idx[ln.parent.name] + 1, idx[ln.name] + 1,
             clear_hand if ('hand' in ln.name or 'finger' in ln.name)
             else clear_arm, ln)
            for ln in self._robot.links if ln.parent is not None]

        target_topic = self.declare_parameter(
            'target_topic', '/target').value
        solution_topic = self.declare_parameter(
            'solution_topic', '/joint_solution').value
        # best_effort: when an oriented target is out of reach, move to the
        # NEAREST reachable pose (closest position, best orientation) instead of
        # refusing to move. Off by default (strict).
        self._best_effort = self.declare_parameter('best_effort', False).value

        # Seed for the next IK call / servo start. Must match where the arm
        # actually IS: the joint_state_translator starts every arm at the zero
        # configuration, so start there too. (Seeding from qr instead made the
        # first planned trajectory begin at a posture whose elbow is INSIDE the
        # relay demo's table — the arm visibly swept through it on startup.)
        self._last_q = np.zeros(7)

        # URDF joint limits of the 7 arm joints, enforced on every published
        # configuration. ikine_LM checks limits itself (joint_limits=True is
        # its default), but our tolerance-based acceptance ignores its success
        # flag — so without this gate, out-of-range solutions (joints folded
        # far past their hard stops, links visually intersecting) get through.
        self._qlim = np.array(
            [ln.qlim for ln in self._robot.links if ln.isjoint][:7])  # (7, 2)
        # Keep the base yaw (joint1) in its forward half so the arm can never
        # reach a pose by swinging ~180° to face backward. (Base joint only.)
        self._qlim[0] = [0.0, np.pi]

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
                # The servo integrates continuously from _last_q and is clamped
                # to the joint limits, so its final point is in-range as-is —
                # no unwrapping (a 2π shift could leave the limit interval).
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
        #
        # With a keep-out volume set, a solution must ALSO be collision-safe:
        # IK on this redundant arm lands on different posture branches per
        # seed, and elbow-below-the-table branches are kinematically valid —
        # so each candidate (and the joint-space sweep the translator will
        # play to reach it) is depth-checked, and violating branches trigger
        # retries from perturbed seeds.
        rng = np.random.default_rng(0)
        # Extra seeds beyond (last, qr): candidates can now be rejected for
        # limit violations too, so always keep a few retries in hand.
        n_seeds = KEEPOUT_IK_SEEDS if self._keepout is not None else 4
        best_q, best_pos, best_ori, best_soft, best_key = None, None, None, None, None
        for i in range(n_seeds):
            if i == 0:
                seed = self._last_q
            elif i == 1:
                seed = self._robot.qr[:7]
            else:   # hunt other branches around the best solution so far
                ref = best_q if best_q is not None else self._last_q
                seed = np.clip(ref + rng.normal(0.0, 0.6, 7),
                               self._qlim[:, 0], self._qlim[:, 1])
            sol = self._robot.ikine_LM(
                T_base, q0=seed, end=self._ee, mask=mask, ilimit=300)
            # Enforce the URDF joint limits: map to the (unique) in-range 2π-
            # equivalent, rejecting genuinely out-of-range solutions. This also
            # kills ~360° spins — the in-range value IS the sane representative.
            q_sol = self._into_limits(sol.q[:7])
            if q_sol is None:
                continue    # joint past its hard stop — try another seed
            pos_mm, ori_deg = self._pose_error(q_sol, T_base, constrain)
            soft = -np.inf
            if self._keepout is not None:
                soft, _ = self._swept_depth(self._last_q, q_sol)
            # Rank: clearance first; then prefer ACCURATE solutions; then, among
            # equally-accurate ones, the posture CLOSEST to where the arm is now
            # (minimal joint travel). This is what stops the redundant base from
            # flipping ~180° between branches to reach the same point.
            accurate = (pos_mm <= POS_TOL_MM
                        and (not constrain or ori_deg <= ORI_TOL_DEG))
            travel = np.linalg.norm(q_sol - self._last_q)
            key = (max(soft, 0.0),
                   0.0 if accurate else pos_mm + (ori_deg or 0),
                   travel)
            if best_key is None or key < best_key:
                best_q, best_pos, best_ori, best_soft, best_key = \
                    q_sol, pos_mm, ori_deg, soft, key
            # No early break: evaluate every seed so the nearest branch wins.

        if best_q is None:
            self.get_logger().error(
                f'IK failed for world target {world_xyz} — every solution '
                f'violates a joint limit')
            return

        ori_msg = f' | ori error: {best_ori:.1f}°' if constrain else ''
        clear_msg = ''
        if self._keepout is not None and best_soft > 0:
            clear_msg = f' | keep-out margin violated by {best_soft * 1000:.0f} mm'
        if (best_pos <= POS_TOL_MM and (not constrain or best_ori <= ORI_TOL_DEG)
                and not clear_msg):
            self.get_logger().info(
                f'IK solved → pos error: {best_pos:.1f} mm{ori_msg}')
            self._last_q = best_q   # seed next call with this solution
            self._joint_pub.publish(Float64MultiArray(data=best_q.tolist()))
        elif self._best_effort:
            self.get_logger().warn(
                f'IK best-effort for world target {world_xyz} '
                f'(closest: {best_pos:.0f} mm{ori_msg}{clear_msg}) '
                f'— moving to nearest reachable pose')
            self._last_q = best_q
            self._joint_pub.publish(Float64MultiArray(data=best_q.tolist()))
        else:
            self.get_logger().error(
                f'IK failed for world target {world_xyz} '
                f'(best: {best_pos:.0f} mm{ori_msg}{clear_msg}) — may be out of reach')

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

    def _into_limits(self, q):
        """Map each joint to its 2π-equivalent inside the URDF limits.

        Every joint's range spans < 360°, so the in-range representative is
        unique — this both enforces the hard stops and kills the ~360° spins
        (a wrapped IK answer like +5.6 rad becomes -0.68 rad if THAT is the
        in-range value). Returns None if some joint has no in-range
        equivalent, i.e. the solution genuinely violates a limit.
        """
        q = np.asarray(q, float).copy()
        for j in range(7):
            lo, hi = self._qlim[j]
            q[j] -= 2 * np.pi * np.floor((q[j] - lo) / (2 * np.pi))
            # q[j] is now the smallest equivalent >= lo; check it's <= hi
            if q[j] > hi + 1e-9:
                return None
        return q

    # --- Workspace keep-out ------------------------------------------------
    def _keepout_worst(self, q):
        """Deepest sampled link-point penetration into the keep-out volume.

        Returns (soft, hard, link, p_link):
          soft   — penetration past each segment's clearance-expanded slab;
                   > 0 means a link is within its safety margin of the table.
          hard   — penetration past the slab top itself (actual contact of the
                   link skeleton); > 0 means the arm is IN the table.
          link, p_link — the worst point's link and its coordinates in that
                   link's frame, for the avoidance Jacobian. None when every
                   sampled point is clear of the (expanded) footprint.
        """
        xmin, xmax, ymin, ymax, ztop = self._keepout
        qq = np.concatenate([np.asarray(q, float), [0.0, 0.0]])
        Ts = self._robot.fkine_all(qq)
        origins = np.array([T.t for T in Ts])
        world = (self._base_pose.R @ origins.T).T + self._base_pose.t
        soft, hard, worst = -np.inf, -np.inf, None
        for pi, ci, margin, ln in self._segments:
            p0, p1 = world[pi], world[ci]
            for f in np.linspace(0.0, 1.0, KEEPOUT_SAMPLES):
                pt = p0 + f * (p1 - p0)
                if not (xmin - margin <= pt[0] <= xmax + margin and
                        ymin - margin <= pt[1] <= ymax + margin):
                    continue
                if xmin <= pt[0] <= xmax and ymin <= pt[1] <= ymax:
                    hard = max(hard, ztop - pt[2])
                depth = (ztop + margin) - pt[2]
                if depth > soft:
                    soft, worst = depth, (pi, ci, f, ln)
        if worst is None:
            return -np.inf, -np.inf, None, None
        pi, ci, f, ln = worst
        # The worst point sits at fraction f from the parent frame toward the
        # link's own frame; express it in the link frame for jacob0(tool=...).
        parent_in_link = (Ts[ci].inv() * Ts[pi]).t
        return soft, hard, ln, (1.0 - f) * parent_in_link

    def _keepout_avoid_dq(self, q, soft, link, p_link):
        """Joint velocity that raises the worst keep-out point (world +z)."""
        qq = np.concatenate([np.asarray(q, float), [0.0, 0.0]])
        J0 = self._robot.jacob0(qq, end=link, tool=SE3(*p_link))
        zrow = (self._base_pose.R @ J0[:3, :])[2]   # world ż per chain joint
        g = np.zeros(7)
        m = min(zrow.shape[0], 7)                   # finger chains have 8 cols
        g[:m] = zrow[:m]
        return KEEPOUT_GAIN * (soft + KEEPOUT_ACTIVATE) * g

    def _swept_depth(self, q_from, q_to, steps=8):
        """Worst (soft, hard) keep-out depth along the joint-space line the
        translator will play between two configurations. Both endpoints are
        within the joint limits (an interval), so the straight line between
        them is too — no unwrapping."""
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        soft, hard = -np.inf, -np.inf
        for f in np.linspace(0.0, 1.0, steps):
            s, h, _, _ = self._keepout_worst(q_from + f * (q_to - q_from))
            soft, hard = max(soft, s), max(hard, h)
        return soft, hard

    def _resolved_rate_traj(self, q_start, T_goal,
                            gain=1.5, dt=0.05, max_iter=400):
        """Resolved-rate servo from q_start to the goal pose, returning the
        joint trajectory (list of 7-vectors) that drives the TCP there along a
        straight Cartesian line at constant orientation.

        Uses the BODY-frame Jacobian (jacobe) to match p_servo's body-frame
        velocity twist, with damped least squares for stability near
        singularities. When a keep-out volume is set, links nearing it are
        steered away through the null space of the end-effector task, so the
        TCP stays on its straight line while the elbow lifts clear. Returns
        None if it fails to converge in tolerance or a link still ends up
        inside the keep-out volume.
        """
        q = np.concatenate([np.asarray(q_start, float), [0.0, 0.0]])  # pad grip
        traj = [q[:7].copy()]
        arrived = False
        for _ in range(max_iter):
            Te = self._robot.fkine(q, end=self._ee)
            v, arrived = p_servo(Te, T_goal, gain=gain, threshold=0.002)
            J = self._robot.jacobe(q, end=self._ee)          # 6x7, body frame
            M = np.linalg.inv(J @ J.T + 0.0025 * np.eye(6))  # damped (JJᵀ+λI)⁻¹
            dq = J.T @ (M @ v)                               # DLS
            if self._keepout is not None:
                soft, _, link, p_link = self._keepout_worst(q[:7])
                if link is not None and soft > -KEEPOUT_ACTIVATE:
                    g = self._keepout_avoid_dq(q[:7], soft, link, p_link)
                    N = np.eye(7) - J.T @ (M @ J)            # null-space projector
                    dq = dq + N @ g
            q = q.copy()
            # Integrate, clamped to the URDF joint limits — the servo must
            # never command a joint past its hard stop (links visually fold
            # through each other there). If clamping stalls progress, the
            # arrival/tolerance checks below fail and the planner falls back.
            q[:7] = np.clip(q[:7] + dq * dt,
                            self._qlim[:, 0], self._qlim[:, 1])
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
        traj = self._resample_arclength(traj, n=40)
        if self._keepout is not None:
            hard = max(self._keepout_worst(wq)[1] for wq in traj)
            if hard > 0:
                self.get_logger().warn(
                    f'Servo trajectory enters the keep-out volume '
                    f'({hard * 1000:.0f} mm) despite null-space avoidance — '
                    f'rejecting it')
                return None
        return traj

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
