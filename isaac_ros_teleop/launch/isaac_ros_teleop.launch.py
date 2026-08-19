# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

"""Launch the IsaacTeleop ROS 2 publisher node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
            name='head_pose_topic',
            default_value='xr_teleop/head_pose',
            description='Topic for head pose command (PoseStamped)',
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
            name='head_frame',
            default_value='head',
            description='TF child frame name for the head',
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
        DeclareLaunchArgument(
            name='cloudxr_install_dir',
            default_value=[
                EnvironmentVariable('ISAAC_ROS_WS', default_value='~'),
                '/.cloudxr',
            ],
            description='CloudXR install directory used by the in-process launcher.',
        ),
        DeclareLaunchArgument(
            name='cloudxr_env_config',
            default_value='',
            description='Optional CloudXR env config file passed to the in-process launcher.',
        ),
        DeclareLaunchArgument(
            name='cloudxr_accept_eula',
            default_value='false',
            description='Accept the NVIDIA CloudXR EULA non-interactively.',
        ),
        DeclareLaunchArgument(
            name='cloudxr_setup_oob',
            default_value='false',
            description='Enable the CloudXR out-of-band teleop control hub.',
        ),
        DeclareLaunchArgument(
            name='cloudxr_usb_local',
            default_value='false',
            description='Route teleop traffic over USB through adb reverse.',
        ),
        DeclareLaunchArgument(
            name='pose_reset_config',
            default_value='',
            description='pose_reset_node parameter file; empty disables the node.',
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use the simulation clock for teleop nodes (true under sim).',
        ),
    ]

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    teleop_publisher_node = Node(
        package='isaac_teleop_core',
        executable='teleop_ros2_node',
        name='teleop_ros2_node',
        parameters=[{
            'rate_hz': LaunchConfiguration('rate_hz'),
            'world_frame': LaunchConfiguration('world_frame'),
            'right_wrist_frame': LaunchConfiguration('right_wrist_frame'),
            'left_wrist_frame': LaunchConfiguration('left_wrist_frame'),
            'head_frame': LaunchConfiguration('head_frame'),
            'left_finger_joint_names': LaunchConfiguration('left_finger_joint_names'),
            'right_finger_joint_names': LaunchConfiguration('right_finger_joint_names'),
            'transform_rotation': LaunchConfiguration('transform_rotation'),
            'transform_translation': LaunchConfiguration('transform_translation'),
            'cloudxr_install_dir': LaunchConfiguration('cloudxr_install_dir'),
            'cloudxr_env_config': LaunchConfiguration('cloudxr_env_config'),
            'cloudxr_accept_eula': LaunchConfiguration('cloudxr_accept_eula'),
            'cloudxr_setup_oob': LaunchConfiguration('cloudxr_setup_oob'),
            'cloudxr_usb_local': LaunchConfiguration('cloudxr_usb_local'),
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('xr_teleop/ee_poses', LaunchConfiguration('ee_pose_topic')),
            ('xr_teleop/root_twist', LaunchConfiguration('root_twist_topic')),
            ('xr_teleop/root_pose', LaunchConfiguration('root_pose_topic')),
            ('xr_teleop/head_pose', LaunchConfiguration('head_pose_topic')),
            ('xr_teleop/finger_joints', LaunchConfiguration('finger_joints_topic')),
            ('xr_teleop/controller_data', LaunchConfiguration('controller_data_topic')),
        ],
        output='screen',
    )

    # pose_reset_node owns world_frame -> robot-root TF; parent single-sourced from
    # world_frame, child frame + tuning from pose_reset_config. Skipped if no config given.
    pose_reset_node = Node(
        package='isaac_ros_teleop',
        executable='pose_reset_node',
        name='pose_reset_node',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('pose_reset_config'), "' != ''"])),
        parameters=[
            LaunchConfiguration('pose_reset_config'),
            {
                'parent_frame_id': LaunchConfiguration('world_frame'),
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription(declared_arguments + [teleop_publisher_node, pose_reset_node])
