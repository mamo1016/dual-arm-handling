"""
pick_place_relay.py — two arms relay one parcel between two table spots.

Arm A picks the parcel at point A, lifts it straight up 10 cm, carries it over to
point B, and lowers it straight down onto B. Arm B then picks it up at B, carries
it back, and lowers it onto A. Repeat forever.

Grasps and places are TOP-DOWN: at the table the gripper's approach (z/blue)
axis points straight at the ground. A vertical grip across the WHOLE path is
not reachable within the URDF joint limits (the lift tops put the wrist
outside its envelope), so the lift/carry waypoints use a slightly tilted grip
(20° toward the arm's own base) and the resolved-rate servo blends the
orientation along the straight vertical lift/lower — the hand "verticalizes"
as it descends. To bring both table spots inside the vertical-grasp envelope
WITH every joint inside its hard stops, the arms are mounted on 25 cm
pedestals 0.7 m apart (found by feasibility search over IK + joint limits +
table keep-out).

Both the lift and the lower are TRUE vertical straight lines in position, and
only ONE arm is ever near the A–B corridor at a time (strict alternation), so
the two can't collide.

Pipeline (per arm):
    pick_place_relay ──/arm_X/target (Pose)──▶ arm_planner_X ──▶
        /arm_X/joint_solution ──▶ joint_state_translator_X ──▶
        /joint_states ──▶ robot_state_publisher ──TF──▶ rviz2

The parcel is a marker that tracks the carrying arm's gripper TCP while carried,
and rests on the table at A or B otherwise — no physics engine needed.

Run:
    ros2 launch dual_arm pick_place_relay.launch.py
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
# Arm bases — EXPORTED: the launch file imports these (single source of truth).
# Each arm stands on a 25 cm pedestal, 0.7 m apart, facing the other. The
# pedestal height, pick-line offset and spot separation come from a
# feasibility search WITH the URDF joint limits enforced: they are the minimal
# scene that puts BOTH table spots inside the vertical-grasp envelope of BOTH
# arms with every joint inside its hard stops. (The wrist pitch is only ±45°,
# which forces the forearm near-vertical above a top-down grasp and caps how
# far out a vertical grasp can reach — ground-mounted arms can't do it at all.)
PEDESTAL_H = 0.25
PEDESTAL_SIZE = (0.14, 0.14, PEDESTAL_H)
ARM_A_XYZ, ARM_A_RPY = (0.0, 0.0, PEDESTAL_H), (0.0, 0.0, 0.0)
ARM_B_XYZ, ARM_B_RPY = (-0.7, 0.0, PEDESTAL_H), (0.0, 0.0, math.pi)

TABLE_POS  = (-0.35, 0.10, 0.24)                   # centred between the bases
TABLE_SIZE = (0.40, 0.40, 0.02)
TABLE_TOP  = TABLE_POS[2] + TABLE_SIZE[2] / 2      # 0.25
PARCEL_SIZE = (0.06, 0.06, 0.06)

LIFT = 0.10                                        # vertical pick/place travel (10 cm)
Z_PICK = TABLE_TOP + PARCEL_SIZE[2] / 2            # parcel centre resting on the table

# Two table spots the parcel is relayed between, and the point 10 cm above each.
A = (-0.31, 0.05, Z_PICK)
B = (-0.39, 0.05, Z_PICK)
A_UP = (A[0], A[1], A[2] + LIFT)
B_UP = (B[0], B[1], B[2] + LIFT)

# Idle/rest poses, each tucked near its own arm's base (same offset relative to
# the base as before the pedestals), well clear of the A–B corridor so the
# waiting arm never fouls the working one.
A_HOME = (ARM_A_XYZ[0] - 0.18, 0.20, PEDESTAL_H + 0.40)
B_HOME = (ARM_B_XYZ[0] + 0.18, 0.20, PEDESTAL_H + 0.40)

# Gripper orientations (w, x, y, z), found by the same limit-aware search:
#   *_DOWN — TRUE top-down grasp: the TCP's approach (z/blue) axis points
#            straight at the ground (arm-specific yaw about the vertical:
#            +30° for arm_a, -60° for arm_b — the yaw is what lets the ±45°
#            wrist pitch reach the pose within its hard stops).
#   *_UP   — the same grip tilted 20° toward the arm's own base. The lift tops
#            are NOT reachable with a perfectly vertical wrist, so each lift/
#            lower blends orientation along its straight vertical line — the
#            hand is exactly vertical the moment it grasps or releases.
# (None = zero quaternion = the planner's "orientation free", for home moves.)
GRIP_A_DOWN = (0.0, 0.866025, -0.5, 0.0)
GRIP_B_DOWN = (0.0, 0.258819, 0.965926, 0.0)
GRIP_A_UP   = (0.150384, 0.852869, -0.492404, -0.086824)
GRIP_B_UP   = (0.044943, 0.254887, 0.951251, 0.167731)

# Twelve-phase cycle: (name, arm_a target, arm_a grip, arm_b target, arm_b grip).
# Arm A relays A→B (phases 1–6) while arm B waits at B_HOME; then arm B relays
# B→A (phases 7–12) while arm A waits at A_HOME.
#   reach  — bring the gripper down onto the parcel, hand vertical
#   lift   — straight up 10 cm, tilting to the carry grip (carried)
#   carry  — across to above the far spot (carried)
#   place  — straight down 10 cm, verticalizing; release onto the table
#   clear  — empty hand straight back up, off the parcel
#   home   — return to the rest pose, ceding the corridor to the other arm
PHASES = [
    ('a_reach', A,      GRIP_A_DOWN, B_HOME, None),
    ('a_lift',  A_UP,   GRIP_A_UP,   B_HOME, None),
    ('a_carry', B_UP,   GRIP_A_UP,   B_HOME, None),
    ('a_place', B,      GRIP_A_DOWN, B_HOME, None),
    ('a_clear', B_UP,   GRIP_A_UP,   B_HOME, None),
    ('a_home',  A_HOME, None,        B_HOME, None),
    ('b_reach', A_HOME, None,        B,      GRIP_B_DOWN),
    ('b_lift',  A_HOME, None,        B_UP,   GRIP_B_UP),
    ('b_carry', A_HOME, None,        A_UP,   GRIP_B_UP),
    ('b_place', A_HOME, None,        A,      GRIP_B_DOWN),
    ('b_clear', A_HOME, None,        A_UP,   GRIP_B_UP),
    ('b_home',  A_HOME, None,        B_HOME, None),
]

# Where the parcel is drawn each phase.
CARRIED_BY_A = {'a_lift', 'a_carry', 'a_place'}
CARRIED_BY_B = {'b_lift', 'b_carry', 'b_place'}
PARCEL_AT_A  = {'a_reach', 'b_clear', 'b_home'}    # resting on A
PARCEL_AT_B  = {'a_clear', 'a_home', 'b_reach'}    # resting on B

CYCLE_SECONDS = 1.8        # time per phase (must exceed the arm's anim_duration)
SCENE_HZ      = 20.0       # marker publish rate — parcel must look smooth


class PickPlaceRelay(Node):
    def __init__(self):
        super().__init__('pick_place_relay')

        self._pub_a = self.create_publisher(Pose, '/arm_a/target', 10)
        self._pub_b = self.create_publisher(Pose, '/arm_b/target', 10)
        self._scene_pub = self.create_publisher(MarkerArray, '/scene/markers', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._phase_idx = -1            # first tick advances to 0 ('a_reach')
        self._last_parcel = A           # parcel starts resting on A

        self.create_timer(CYCLE_SECONDS, self._tick)
        self.create_timer(1.0 / SCENE_HZ, self._publish_scene)
        self.get_logger().info('Pick-and-place relay (A↔B) started')

    @property
    def _phase(self):
        return PHASES[self._phase_idx][0]

    def _tick(self):
        self._phase_idx = (self._phase_idx + 1) % len(PHASES)
        name, a_xyz, a_grip, b_xyz, b_grip = PHASES[self._phase_idx]
        self._publish_target(self._pub_a, a_xyz, a_grip)
        self._publish_target(self._pub_b, b_xyz, b_grip)
        self.get_logger().info(f'→ {name}')
        self._publish_scene()

    def _publish_target(self, pub, xyz, quat):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = xyz
        if quat is None:
            # Zero quaternion = the planner's "orientation free" sentinel
            # (geometry_msgs defaults w to 1.0, so we must override it).
            msg.orientation.w = 0.0
        else:
            (msg.orientation.w, msg.orientation.x,
             msg.orientation.y, msg.orientation.z) = quat
        pub.publish(msg)

    # --- Scene: table + the one parcel -------------------------------------
    def _publish_scene(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        arr.markers.append(self._make_box(
            mid=0, ns='furniture', pos=TABLE_POS, size=TABLE_SIZE,
            color=ColorRGBA(r=0.55, g=0.35, b=0.20, a=1.0), stamp=now))

        # The two pedestal columns the arms stand on.
        for mid, base in ((1, ARM_A_XYZ), (2, ARM_B_XYZ)):
            arr.markers.append(self._make_box(
                mid=mid, ns='furniture',
                pos=(base[0], base[1], PEDESTAL_H / 2), size=PEDESTAL_SIZE,
                color=ColorRGBA(r=0.45, g=0.45, b=0.50, a=1.0), stamp=now))

        px, py, pz = self._parcel_pos()
        self._last_parcel = (px, py, pz)
        arr.markers.append(self._make_box(
            mid=0, ns='parcel', pos=(px, py, pz), size=PARCEL_SIZE,
            color=ColorRGBA(r=0.95, g=0.55, b=0.15, a=1.0), stamp=now))

        self._scene_pub.publish(arr)

    def _parcel_pos(self):
        """Where to draw the parcel: tracking the carrying arm's TCP while
        carried, resting on A or B otherwise."""
        phase = self._phase
        if phase in CARRIED_BY_A or phase in CARRIED_BY_B:
            frame = 'arm_a_openarm_hand_tcp' if phase in CARRIED_BY_A \
                else 'arm_b_openarm_hand_tcp'
            try:
                tf = self._tf_buffer.lookup_transform(
                    'world', frame, RclpyTime(), timeout=Duration(seconds=0.05))
                return (tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z)
            except Exception:
                return self._last_parcel
        if phase in PARCEL_AT_A:
            return A
        if phase in PARCEL_AT_B:
            return B
        return self._last_parcel

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
    node = PickPlaceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
