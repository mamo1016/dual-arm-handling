# Dual-Arm Manipulation in ROS 2 🤖🤖

Two 7-DOF [OpenArm](https://github.com/enactic/openarm) manipulators sharing one
workspace, driven by a self-contained inverse-kinematics and Cartesian-control
stack — no MoveIt, no physics engine. Two coordinators run on the same pipeline:

- **`pick_place_relay`** — a pick-and-place relay: two pedestal-mounted arms hand
  a parcel back and forth between two table spots, each grasping and placing
  **top-down** (gripper pointing straight down) with collision-aware, minimal-motion
  joint paths.
- **`hot_potato`** — a light-hearted showcase: the two arms fling one glowing cube
  hand-to-hand, faster and faster, as if it were too hot to hold.

<!-- Record the rviz2 window and drop a clip in here:
     ![relay demo](media/pick_place_relay.gif) -->
> 📹 _Demo clip: add `media/pick_place_relay.gif` (or drag an `.mp4` into this README on github.com)._

---

## What it demonstrates

- **Forward / inverse kinematics** on a redundant 7-DOF arm with
  [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
  (Levenberg–Marquardt), with tolerance-based solution acceptance that judges a
  solve by its achieved-pose error rather than the solver's `success` flag.
- **Singularity-robust Cartesian control** — resolved-rate (Jacobian) servoing
  with damped least squares for straight-line, constant-orientation moves (used
  for the true-vertical lifts and lowers).
- **Workspace keep-out** — a table volume constrains the *whole arm*, not just the
  TCP: IK branches that would sweep a link through the table are rejected, and the
  servo steers offending links away through the null space of the end-effector task.
- **Joint-limit-aware, minimal-motion solutions** — every configuration is mapped
  to its unique in-range 2π-equivalent and gated against the URDF joint limits
  (the servo integrates clamped to them), and IK candidates are ranked by joint
  travel from the current pose. Result: the arm never folds a joint past its hard
  stop and never makes a redundant ~180° base swing to reach a point it could
  reach with a small move — the disciplined motion you see on a factory line.
- **Collision-aware bimanual coordination** — strict alternation keeps one arm in
  the shared corridor at a time, and the idle arm tucks back over its own base
  (idle poses chosen by a clearance search) so the working arm never fouls it.
- **Two arms in one scene** — a combined two-arm URDF is generated
  programmatically from the single-arm description (name-prefixing that preserves
  mesh paths), spliced under a shared `world` frame.

## Architecture

```
coordinator ──/arm_X/target (Pose)──▶ arm_planner_X  (IK) ──▶
   /arm_X/joint_solution ──▶ joint_state_translator_X ──▶
   /joint_states ──▶ robot_state_publisher ──TF──▶ rviz2
```

Each arm reuses the same per-arm pipeline; a coordinator (`pick_place_relay` or
`hot_potato`) only publishes Cartesian targets. The carried object is an rviz
marker that tracks whichever gripper's TCP currently "holds" it — so pick-place
and toss-and-catch need no physics engine.

| File | Role |
|------|------|
| `dual_arm/pick_place_relay.py` | Pick-and-place relay coordinator (top-down grasps) |
| `dual_arm/hot_potato.py` | Bimanual handoff coordinator (the accelerating gag) |
| `dual_arm/arm_planner.py` | Per-arm IK, keep-out, joint limits + resolved-rate Cartesian trajectories |
| `dual_arm/joint_state_translator.py` | Eases joint solutions onto `/joint_states` |
| `dual_arm/dual_urdf.py` | Builds the two-arm URDF from the single-arm one |
| `dual_arm/openarm.py` | Minimal URDF loader for the OpenArm model |
| `tools/verify_relay.py` | Headless check: replays the relay through the real planner, asserts table clearance, joint limits, and solve timing |

## Prerequisites

- **ROS 2** (developed on Jazzy; Humble/Iron should work).
- The **OpenArm description** package on your `ROS_PACKAGE_PATH` — the URDF
  references its meshes via `package://openarm_description/...`:
  ```bash
  cd ~/ros2_ws/src
  git clone https://github.com/enactic/openarm_description.git   # or the OpenArm description repo
  ```
- **Python deps** for the kinematics, in a virtualenv at the repo root:
  ```bash
  python3 -m venv .venv
  ./.venv/bin/pip install -r requirements.txt
  ```
  The package auto-adds a repo-root `.venv` to the node path (see
  `dual_arm/__init__.py`); alternatively put these packages on your `PYTHONPATH`.

## Build & run

```bash
# from your ROS 2 workspace (e.g. ~/ros2_ws), with this repo under src/
colcon build --symlink-install --packages-select dual_arm
source install/setup.bash

ros2 launch dual_arm pick_place_relay.launch.py   # the pick-and-place relay
# or
ros2 launch dual_arm hot_potato.launch.py         # the hot-potato gag
```

rviz2 opens with both arms running the chosen demo. To capture a clip,
screen-record the rviz2 window, then optionally down-convert to a GIF:

```bash
ffmpeg -i clip.mp4 -vf "fps=15,scale=720:-1" media/pick_place_relay.gif
```

### Verifying the relay (headless, no GUI)
`tools/verify_relay.py` drives the **real** planner through every phase and
asserts no link enters the table, no joint exceeds its limit, and each solve
fits the cycle budget:

```bash
python3 tools/verify_relay.py a 2   # arm A, 2 cycles
python3 tools/verify_relay.py b 2   # arm B, 2 cycles
```

### Tuning
- **Relay speed** — `ANIM_DURATION` in `launch/pick_place_relay.launch.py` (per-move
  duration) and `CYCLE_SECONDS` in `dual_arm/pick_place_relay.py` (phase tempo);
  keep `CYCLE_SECONDS` a little above `ANIM_DURATION` so each move settles first.
- **Hot-potato gag** — in `dual_arm/hot_potato.py`: lower `PERIOD_MIN` (more
  frantic), raise `CENTER`'s z (bigger toss arc), or lower `RESET_AFTER`.

> **Note:** the launch files force the `d3d12` Mesa driver, a workaround for
> rviz2 freezing under WSLg. On native Linux with a real GPU, remove the
> `additional_env` block in the launch file.

## Credits & license

- Demo code © 2026 Mamoru Ueda, released under the **Apache License 2.0** (see `LICENSE`).
- The robot model (`openarm_single_control.urdf` and the referenced meshes) is
  derived from the open-source **[OpenArm](https://github.com/enactic/openarm)**
  project, also Apache-2.0. All credit for the manipulator design goes to them.
