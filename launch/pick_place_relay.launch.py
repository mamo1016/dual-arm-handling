"""
Launch the PICK-AND-PLACE RELAY demo — two arms relay one parcel between two
table spots (arm A: A→B, arm B: B→A), with true vertical lifts and lowers.

Same dual-arm pipeline as the other demos; the coordinator is
`pick_place_relay`. anim_duration is moderate so the moves read as deliberate
(the parcel is "slowly" lowered into place).

Starts:
- robot_state_publisher  — TF for BOTH arms from the combined URDF
- joint_state_translator ×2 — bridge each arm's /joint_solution → /joint_states
- arm_planner ×2         — per-arm IK (best_effort so the clip never stalls)
- pick_place_relay       — the relay coordinator
- rviz2                  — 3D visualiser (fixed frame: world)
"""
import os
import shutil
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PKG_SHARE = get_package_share_directory('dual_arm')
URDF_PATH = os.path.join(PKG_SHARE, 'openarm_single_control.urdf')
RVIZ_CONFIG = os.path.join(PKG_SHARE, 'rviz', 'hot_potato.rviz')

# Deliberate, smooth moves (longer than the hot-potato handoff).
ANIM_DURATION = 1.5


def generate_launch_description():
    sys.path.insert(0, PKG_SHARE)
    from dual_arm.dual_urdf import build_dual_urdf
    # Scene geometry is single-sourced from the coordinator module: arm base
    # placement (pedestal mounts) and the table keep-out for the planners.
    from dual_arm.pick_place_relay import (
        ARM_A_XYZ, ARM_A_RPY, ARM_B_XYZ, ARM_B_RPY,
        TABLE_POS, TABLE_SIZE, TABLE_TOP)

    urdf_xml = build_dual_urdf(
        URDF_PATH,
        arm_a_xyz=ARM_A_XYZ, arm_a_rpy=ARM_A_RPY,
        arm_b_xyz=ARM_B_XYZ, arm_b_rpy=ARM_B_RPY)

    # Table keep-out volume for the planners ([xmin, xmax, ymin, ymax, z_top],
    # world frame): no arm link may dip below the table top inside its
    # footprint, so elbow-through-the-table IK branches are rejected/steered.
    keepout_box = [
        TABLE_POS[0] - TABLE_SIZE[0] / 2, TABLE_POS[0] + TABLE_SIZE[0] / 2,
        TABLE_POS[1] - TABLE_SIZE[1] / 2, TABLE_POS[1] + TABLE_SIZE[1] / 2,
        TABLE_TOP,
    ]

    rviz_active = tempfile.NamedTemporaryFile(
        prefix='relay_', suffix='.rviz', delete=False).name
    shutil.copy(RVIZ_CONFIG, rviz_active)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_xml}],
        ),

        Node(
            package='dual_arm', executable='arm_planner', name='arm_planner_a',
            parameters=[{
                'base_xyz': list(ARM_A_XYZ), 'base_rpy': list(ARM_A_RPY),
                'target_topic': '/arm_a/target',
                'solution_topic': '/arm_a/joint_solution',
                'best_effort': True,
                'keepout_box': keepout_box,
            }],
        ),
        Node(
            package='dual_arm', executable='joint_state_translator',
            name='joint_state_translator_a',
            parameters=[{
                'joint_prefix': 'arm_a_openarm_',
                'solution_topic': '/arm_a/joint_solution',
                'anim_duration': ANIM_DURATION,
            }],
        ),

        Node(
            package='dual_arm', executable='arm_planner', name='arm_planner_b',
            parameters=[{
                'base_xyz': list(ARM_B_XYZ), 'base_rpy': list(ARM_B_RPY),
                'target_topic': '/arm_b/target',
                'solution_topic': '/arm_b/joint_solution',
                'best_effort': True,
                'keepout_box': keepout_box,
            }],
        ),
        Node(
            package='dual_arm', executable='joint_state_translator',
            name='joint_state_translator_b',
            parameters=[{
                'joint_prefix': 'arm_b_openarm_',
                'solution_topic': '/arm_b/joint_solution',
                'anim_duration': ANIM_DURATION,
            }],
        ),

        # --- The relay coordinator -----------------------------------------
        Node(package='dual_arm', executable='pick_place_relay'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_active],
            # WSLg/llvmpipe can freeze rviz2; force the d3d12 Mesa driver. Drop
            # this additional_env block on native Linux with a real GPU.
            additional_env={
                'MESA_LOADER_DRIVER_OVERRIDE': 'd3d12',
                'GALLIUM_DRIVER': 'd3d12',
            },
        ),
    ])
