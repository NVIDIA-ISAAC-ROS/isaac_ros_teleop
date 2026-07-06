# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Launch the IsaacTeleop ROS 2 publisher node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# G1 TriHand finger joint names in TriHand retargeter order:
# thumb_rotation, thumb_proximal, thumb_distal,
# index_proximal, index_distal, middle_proximal, middle_distal.
_G1_LEFT_FINGER_JOINT_NAMES = (
    '["left_hand_thumb_0_joint","left_hand_thumb_1_joint","left_hand_thumb_2_joint",'
    '"left_hand_index_0_joint","left_hand_index_1_joint",'
    '"left_hand_middle_0_joint","left_hand_middle_1_joint"]'
)
_G1_RIGHT_FINGER_JOINT_NAMES = (
    '["right_hand_thumb_0_joint","right_hand_thumb_1_joint","right_hand_thumb_2_joint",'
    '"right_hand_index_0_joint","right_hand_index_1_joint",'
    '"right_hand_middle_0_joint","right_hand_middle_1_joint"]'
)

# OpenXR (RUB: x-right, y-up, z-back) → ROS (FLU: x-forward, y-left, z-up) rotation
# as a quaternion [qx, qy, qz, qw].
_OPENXR_TO_ROS_ROTATION = '[0.5, -0.5, -0.5, 0.5]'


def generate_launch_description() -> LaunchDescription:
    """Launch the teleop ROS 2 publisher node."""
    # Forward CloudXR OpenXR runtime env vars from host env; fall back to ~/.cloudxr defaults.
    cxr_host_volume = os.environ.get('CXR_HOST_VOLUME_PATH', os.path.expanduser('~/.cloudxr'))
    node_env = {
        'NV_CXR_RUNTIME_DIR': os.environ.get(
            'NV_CXR_RUNTIME_DIR', os.path.join(cxr_host_volume, 'run')
        ),
        'XR_RUNTIME_JSON': os.environ.get(
            'XR_RUNTIME_JSON', os.path.join(cxr_host_volume, 'openxr_cloudxr.json')
        ),
    }

    declared_arguments = [
        DeclareLaunchArgument(
            name='ee_pose_topic',
            default_value='xr_teleop/ee_poses',
            description='Topic for end-effector poses (PoseArray)',
        ),
        DeclareLaunchArgument(
            name='root_twist_topic',
            default_value='xr_teleop/root_twist',
            description='Topic for root velocity command (TwistStamped)',
        ),
        DeclareLaunchArgument(
            name='root_pose_topic',
            default_value='xr_teleop/root_pose',
            description='Topic for root pose command (PoseStamped)',
        ),
        DeclareLaunchArgument(
            name='finger_joints_topic',
            default_value='xr_teleop/finger_joints',
            description='Topic for retargeted TriHand finger joint angles (JointState)',
        ),
        DeclareLaunchArgument(
            name='controller_data_topic',
            default_value='xr_teleop/controller_data',
            description='Topic for raw msgpack-encoded controller state (ByteMultiArray)',
        ),
        DeclareLaunchArgument(
            name='rate_hz',
            default_value='60.0',
            description='Publishing rate in Hz',
        ),
        DeclareLaunchArgument(
            name='world_frame',
            default_value='world',
            description='World frame for message headers and TF parent frame',
        ),
        DeclareLaunchArgument(
            name='right_wrist_frame',
            default_value='right_wrist',
            description='TF child frame name for the right wrist',
        ),
        DeclareLaunchArgument(
            name='left_wrist_frame',
            default_value='left_wrist',
            description='TF child frame name for the left wrist',
        ),
        DeclareLaunchArgument(
            name='left_finger_joint_names',
            default_value=_G1_LEFT_FINGER_JOINT_NAMES,
            description=(
                'Left-hand finger joint names in TriHand retargeter order: '
                'thumb_rotation, thumb_proximal, thumb_distal, '
                'index_proximal, index_distal, middle_proximal, middle_distal.'
            ),
        ),
        DeclareLaunchArgument(
            name='right_finger_joint_names',
            default_value=_G1_RIGHT_FINGER_JOINT_NAMES,
            description=(
                'Right-hand finger joint names in TriHand retargeter order: '
                'thumb_rotation, thumb_proximal, thumb_distal, '
                'index_proximal, index_distal, middle_proximal, middle_distal.'
            ),
        ),
        DeclareLaunchArgument(
            name='transform_rotation',
            default_value=_OPENXR_TO_ROS_ROTATION,
            description=(
                'Quaternion [qx, qy, qz, qw] to rotate XR poses into the ROS world frame. '
                'Defaults to the OpenXR-to-ROS (RUB→FLU) rotation.'
            ),
        ),
        DeclareLaunchArgument(
            name='transform_translation',
            default_value='[0.0, 0.0, 0.0]',
            description=(
                'Translation [x, y, z] applied to XR poses after rotation. '
                'Override from bringup to account for robot-specific world-origin offsets '
                '(e.g. "[0.0, 0.0, -1.0]" when the XR world origin is 1 m below pelvis).'
            ),
        ),
    ]

    teleop_publisher_node = Node(
        package='isaac_teleop_core',
        executable='teleop_ros2_node',
        name='teleop_ros2_node',
        parameters=[{
            'rate_hz': LaunchConfiguration('rate_hz'),
            'world_frame': LaunchConfiguration('world_frame'),
            'right_wrist_frame': LaunchConfiguration('right_wrist_frame'),
            'left_wrist_frame': LaunchConfiguration('left_wrist_frame'),
            'left_finger_joint_names': LaunchConfiguration('left_finger_joint_names'),
            'right_finger_joint_names': LaunchConfiguration('right_finger_joint_names'),
            'transform_rotation': LaunchConfiguration('transform_rotation'),
            'transform_translation': LaunchConfiguration('transform_translation'),
        }],
        remappings=[
            ('xr_teleop/ee_poses', LaunchConfiguration('ee_pose_topic')),
            ('xr_teleop/root_twist', LaunchConfiguration('root_twist_topic')),
            ('xr_teleop/root_pose', LaunchConfiguration('root_pose_topic')),
            ('xr_teleop/finger_joints', LaunchConfiguration('finger_joints_topic')),
            ('xr_teleop/controller_data', LaunchConfiguration('controller_data_topic')),
        ],
        additional_env=node_env,
        output='screen',
    )

    return LaunchDescription(declared_arguments + [teleop_publisher_node])
