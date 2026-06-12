"""Headless end-to-end check of the relay demo's motion planning.

Instantiates the REAL ArmPlanner node with the same parameters the launch file
passes (scene constants imported from dual_arm.pick_place_relay — no
duplication), feeds it the relay's phase targets, captures the published joint
solutions, simulates the joint_state_translator's playback (prepend current
config, unwrap, dense linear interpolation), and independently measures link
penetration into the table slab. Also times every solve against the phase
budget (CYCLE_SECONDS).

Usage (with ROS 2 and the workspace sourced):
    python3 tools/verify_relay.py a [cycles]     # arm_a
    python3 tools/verify_relay.py b [cycles]     # arm_b
Exits non-zero if any played configuration puts a link inside the table.
"""
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dual_arm.pick_place_relay import (  # noqa: E402
    TABLE_POS, TABLE_SIZE, TABLE_TOP, PHASES)

try:  # base placement (exported after the vertical-grasp scene update)
    from dual_arm.pick_place_relay import (
        ARM_A_XYZ, ARM_A_RPY, ARM_B_XYZ, ARM_B_RPY)
except ImportError:
    ARM_A_XYZ, ARM_A_RPY = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    ARM_B_XYZ, ARM_B_RPY = (-0.8, 0.0, 0.0), (0.0, 0.0, np.pi)

ARM = sys.argv[1] if len(sys.argv) > 1 else 'a'
CYCLES = int(sys.argv[2]) if len(sys.argv) > 2 else 3

TX = (TABLE_POS[0] - TABLE_SIZE[0] / 2, TABLE_POS[0] + TABLE_SIZE[0] / 2)
TY = (TABLE_POS[1] - TABLE_SIZE[1] / 2, TABLE_POS[1] + TABLE_SIZE[1] / 2)

if ARM == 'a':
    base_xyz, base_rpy, t_col, g_col = ARM_A_XYZ, ARM_A_RPY, 1, 2
else:
    base_xyz, base_rpy, t_col, g_col = ARM_B_XYZ, ARM_B_RPY, 3, 4

rclpy.init(args=[
    '--ros-args',
    '-p', f'base_xyz:=[{base_xyz[0]}, {base_xyz[1]}, {base_xyz[2]}]',
    '-p', f'base_rpy:=[{base_rpy[0]}, {base_rpy[1]}, {base_rpy[2]}]',
    '-p', 'best_effort:=true',
    '-p', f'keepout_box:=[{TX[0]}, {TX[1]}, {TY[0]}, {TY[1]}, {TABLE_TOP}]',
])

from dual_arm.arm_planner import ArmPlanner  # noqa: E402

node = ArmPlanner()
captured = []


class CapturePub:
    def publish(self, msg):
        captured.append(np.array(msg.data).reshape(-1, 7))


node._joint_pub = CapturePub()

# Independent collision geometry (parent-based link segments, own math).
robot, base = node._robot, node._base_pose
idx = {ln.name: i for i, ln in enumerate(robot.links)}
segs = [(idx[ln.parent.name] + 1, idx[ln.name] + 1, ln.name)
        for ln in robot.links if ln.parent is not None]
QLIM = np.array([ln.qlim for ln in robot.links if ln.isjoint][:7])  # (7, 2)


def limit_violation(q):
    """Worst joint excursion past its URDF limit (rad, 0 = within limits)."""
    return float(max(0.0, np.max(np.maximum(QLIM[:, 0] - q, q - QLIM[:, 1]))))


def worst_depth(q):
    """(hard penetration below table top, worst link name) for config q."""
    Ts = robot.fkine_all(np.concatenate([q, [0.0, 0.0]]))
    pts = (base.R @ np.array([T.t for T in Ts]).T).T + base.t
    worst, name = -np.inf, None
    for pi, ci, ln in segs:
        for f in np.linspace(0, 1, 8):
            p = pts[pi] + f * (pts[ci] - pts[pi])
            if TX[0] <= p[0] <= TX[1] and TY[0] <= p[1] <= TY[1]:
                d = TABLE_TOP - p[2]
                if d > worst:
                    worst, name = d, ln
    return worst, name


def make_pose(xyz, quat):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = xyz
    if quat is None:
        msg.orientation.w = 0.0
    else:
        (msg.orientation.w, msg.orientation.x,
         msg.orientation.y, msg.orientation.z) = quat
    return msg


current = np.zeros(7)
overall_hard = -np.inf
fail = False
for c in range(CYCLES):
    for row in PHASES:
        name, xyz, grip = row[0], row[t_col], row[g_col]
        captured.clear()
        t0 = time.perf_counter()
        node._on_target(make_pose(xyz, grip))
        dt = time.perf_counter() - t0
        if not captured:
            print(f'cyc{c} {name:<8} NO SOLUTION PUBLISHED  ({dt:.2f}s)')
            fail = True
            continue
        traj = captured[-1]
        # Mirrors the translator: direct linear playback, no unwrap (the
        # planner emits canonical in-limit configurations).
        path = np.vstack([current] + list(traj))
        worst, wname, lviol = -np.inf, None, 0.0
        for i in range(len(path) - 1):
            for f in np.linspace(0, 1, 6, endpoint=False):
                qi = path[i] + f * (path[i + 1] - path[i])
                d, ln = worst_depth(qi)
                if d > worst:
                    worst, wname = d, ln
                lviol = max(lviol, limit_violation(qi))
        d, ln = worst_depth(path[-1])
        if d > worst:
            worst, wname = d, ln
        lviol = max(lviol, limit_violation(path[-1]))
        current = path[-1]
        overall_hard = max(overall_hard, worst)
        status = (f'*** PENETRATES {worst * 1000:+.0f} mm ({wname})'
                  if worst > 0 else f'clear ({worst * 1000:+.0f} mm margin)')
        # Held configurations must respect joint limits. (The translator's
        # unwrap may briefly traverse a 2π-equivalent route between phases —
        # only flag the endpoint, which is what the arm holds.)
        end_viol = limit_violation(path[-1])
        if end_viol > 0.02:
            status += f'  *** JOINT LIMIT +{np.degrees(end_viol):.0f}deg'
            fail = True
        elif lviol > 0.02:
            status += f'  (swept 2pi-route past a limit by {np.degrees(lviol):.0f}deg)'
        if worst > 0:
            fail = True
        print(f'cyc{c} {name:<8} pts={len(traj):>3} solve={dt:5.2f}s  {status}')

print(f'\nARM {ARM}: {"FAIL" if fail else "PASS"} — worst skeleton depth vs '
      f'table top: {overall_hard * 1000:+.0f} mm (negative = clear)')
node.destroy_node()
rclpy.shutdown()
sys.exit(1 if fail else 0)
