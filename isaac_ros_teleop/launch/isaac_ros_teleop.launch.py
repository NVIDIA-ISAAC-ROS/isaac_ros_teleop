# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the teleop ROS 2 publisher node."""
    launch_args = [
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
            name='finger_joints_remapped_topic',
            default_value='xr_teleop/finger_joints_remapped',
            description='Output topic for remapped finger joint names (JointState)',
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
    ]

    launch_configs = {
        'ee_pose_topic': LaunchConfiguration('ee_pose_topic'),
        'root_twist_topic': LaunchConfiguration('root_twist_topic'),
        'root_pose_topic': LaunchConfiguration('root_pose_topic'),
        'finger_joints_topic': LaunchConfiguration('finger_joints_topic'),
        'finger_joints_remapped_topic': LaunchConfiguration('finger_joints_remapped_topic'),
        'rate_hz': LaunchConfiguration('rate_hz'),
        'world_frame': LaunchConfiguration('world_frame'),
        'right_wrist_frame': LaunchConfiguration('right_wrist_frame'),
        'left_wrist_frame': LaunchConfiguration('left_wrist_frame'),
    }

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

    teleop_publisher_node = Node(
        package='isaac_teleop_core',
        executable='teleop_ros2_publisher',
        name='teleop_ros2_publisher',
        parameters=[launch_configs],
        additional_env=node_env,
        output='screen',
    )

    finger_joint_renamer_node = Node(
        package='isaac_ros_teleop',
        executable='finger_joint_renamer',
        name='finger_joint_renamer',
        parameters=[{
            'input_topic': launch_configs['finger_joints_topic'],
            'output_topic': launch_configs['finger_joints_remapped_topic'],
        }],
        output='screen',
    )

    return launch.LaunchDescription(launch_args + [
        teleop_publisher_node,
        finger_joint_renamer_node,
    ])
