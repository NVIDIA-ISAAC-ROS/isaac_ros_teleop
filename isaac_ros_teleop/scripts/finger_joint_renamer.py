#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 node that remaps finger joint names from XR teleop to robot hand convention."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_SUFFIX_MAP = {
    '_thumb_rotation': '_hand_thumb_0_joint',
    '_thumb_proximal': '_hand_thumb_1_joint',
    '_thumb_distal': '_hand_thumb_2_joint',
    '_index_proximal': '_hand_index_0_joint',
    '_index_distal': '_hand_index_1_joint',
    '_middle_proximal': '_hand_middle_0_joint',
    '_middle_distal': '_hand_middle_1_joint',
}


def remap_joint_name(name):
    """Remap a single joint name using the suffix map; unknown names pass through."""
    for suffix, replacement in JOINT_SUFFIX_MAP.items():
        if name.endswith(suffix):
            prefix = name[: -len(suffix)]
            return prefix + replacement
    return name


class FingerJointRenamer(Node):
    """Subscribes to a JointState topic and republishes with remapped joint names."""

    def __init__(self):
        super().__init__('finger_joint_renamer')
        self.declare_parameter('input_topic', 'xr_teleop/finger_joints')
        self.declare_parameter('output_topic', 'xr_teleop/finger_joints_remapped')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(JointState, output_topic, 10)
        self._sub = self.create_subscription(JointState, input_topic, self._callback, 10)

        self.get_logger().info(
            f'finger_joint_renamer: {input_topic} -> {output_topic}'
        )

    def _callback(self, msg):
        out = JointState()
        out.header = msg.header
        out.name = [remap_joint_name(n) for n in msg.name]
        out.position = msg.position
        out.velocity = msg.velocity
        out.effort = msg.effort
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FingerJointRenamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
