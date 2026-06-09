# Dual-Arm Hot Potato 🔥🤖🤖

Two 7-DOF [OpenArm](https://github.com/enactic/openarm) manipulators fling a single
glowing cube back and forth across a shared workspace — faster and faster, as if
it were too hot to hold. A small, self-contained **ROS 2** demo of dual-arm
coordination, inverse kinematics, and singularity-robust Cartesian control.

<!-- Record the rviz2 window and drop the clip in here:
     ![hot potato demo](media/hot_potato.gif) -->
> 📹 _Demo clip: add `media/hot_potato.gif` (or drag an `.mp4` into this README on github.com)._

---

## What it demonstrates

- **Forward / inverse kinematics** on a redundant 7-DOF arm with
  [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
  (Levenberg–Marquardt), with tolerance-based solution acceptance that judges a
  solve by its achieved-pose error rather than the solver's `success` flag.
- **Singularity-robust Cartesian control** — resolved-rate (Jacobian) servoing
  with damped least squares for straight-line, constant-orientation moves.
- **Shortest-path joint motion** — commanded configurations are `unwrap`-ed per
  joint so the arm never spins ~360° to reach a kinematically identical pose.
- **Two arms in one scene** — a combined two-arm URDF is generated
  programmatically from the single-arm description (name-prefixing that preserves
  mesh paths), spliced under a shared `world` frame.
- **A coordinated bimanual handoff** — the `hot_potato` coordinator passes one
  object hand-to-hand with an accelerating tempo.

## Architecture

```
hot_potato ──/arm_X/target (Pose)──▶ arm_planner_X  (IK) ──▶
   /arm_X/joint_solution ──▶ joint_state_translator_X ──▶
   /joint_states ──▶ robot_state_publisher ──TF──▶ rviz2
```

Each arm reuses the same per-arm pipeline; the coordinator only publishes
Cartesian targets. The cube is an rviz marker that tracks whichever gripper's
TCP currently "holds" it — so the toss-and-catch needs no physics engine.

| File | Role |
|------|------|
| `dual_arm/hot_potato.py` | Handoff coordinator (the demo) |
| `dual_arm/arm_planner.py` | Per-arm IK + resolved-rate Cartesian trajectories |
| `dual_arm/joint_state_translator.py` | Eases joint solutions onto `/joint_states` |
| `dual_arm/dual_urdf.py` | Builds the two-arm URDF from the single-arm one |
| `dual_arm/openarm.py` | Minimal URDF loader for the OpenArm model |

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
ros2 launch dual_arm hot_potato.launch.py
```

rviz2 opens with both arms playing hot potato. To capture a clip, screen-record
the rviz2 window (~15–20 s catches a full slow→fast→reset cycle), then optionally:

```bash
ffmpeg -i clip.mp4 -vf "fps=15,scale=720:-1" media/hot_potato.gif
```

### Tuning the gag
In `dual_arm/hot_potato.py`: lower `PERIOD_MIN` (more frantic), raise `CENTER`'s
z (bigger toss arc), or lower `RESET_AFTER` (resets to slow more often).

> **Note:** the launch file forces the `d3d12` Mesa driver, a workaround for
> rviz2 freezing under WSLg. On native Linux with a real GPU, remove the
> `additional_env` block in `launch/hot_potato.launch.py`.

## Credits & license

- Demo code © 2026 Mamoru Ueda, released under the **Apache License 2.0** (see `LICENSE`).
- The robot model (`openarm_single_control.urdf` and the referenced meshes) is
  derived from the open-source **[OpenArm](https://github.com/enactic/openarm)**
  project, also Apache-2.0. All credit for the manipulator design goes to them.
