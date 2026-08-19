// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ISAAC_ROS_TELEOP__POSE_RESET_NODE_HPP_
#define ISAAC_ROS_TELEOP__POSE_RESET_NODE_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace nvidia
{
namespace isaac_ros
{
namespace teleop
{

// PoseArray layout from /xr_teleop/hand is fixed by the upstream teleop_ros2
// publisher — do NOT change these indices.
inline constexpr int kRightThumbTipIdx = 3;
inline constexpr int kRightLittleTipIdx = 23;
inline constexpr int kLeftThumbTipIdx = 27;
inline constexpr int kLeftLittleTipIdx = 47;

// Min XY separation (m) between EE references for a well-defined heading; below is degenerate.
inline constexpr double kMinReferenceSeparationM = 1e-3;

// ---------------------------------------------------------------------------
// Pure helpers (free functions, unit-tested in test/test_pose_reset_node.cpp).
// ---------------------------------------------------------------------------

/// True for a ~zero position and ~identity-quaternion orientation (uninitialized sentinel pose).
bool IsZeroPose(
  const geometry_msgs::msg::Pose & pose, double pos_tol = 1e-3, double rot_tol = 1e-3);

/// Build parent->child from the two EE poses: origin = XY midpoint (Z=0), heading from the line
/// between them, plus a child-frame offset. Returns nullopt if they coincide in XY or non-finite.
/// `left`=poses[0], `right`=poses[1] (/xr_teleop/ee_poses); swapping them yaws the frame 180 deg.
std::optional<Eigen::Isometry3d> ComputeParentPoseChild(
  const geometry_msgs::msg::Pose & left, const geometry_msgs::msg::Pose & right,
  const Eigen::Vector3d & child_frame_offset);

/// Euclidean distance between the positions of two indexed poses in a PoseArray,
/// or nullopt if either index is out of range.
std::optional<float> PositionDistance(
  const geometry_msgs::msg::PoseArray & poses, int idx_a, int idx_b);

/// Outcome of decoding /xr_teleop/controller_data for the configured reset combo.
struct ResetComboState
{
  bool decoded = false;             ///< payload decoded as a msgpack map
  bool any_combo_key_seen = false;  ///< at least one configured combo key present in the map
  bool all_pressed = false;         ///< every combo key present and at/above the press threshold
};

/// Decode controller_data (a msgpack map) and evaluate the configured button
/// combo. Honors ByteMultiArray.layout.data_offset. Pure: takes no node state.
ResetComboState EvaluateResetCombo(
  const std_msgs::msg::ByteMultiArray & msg,
  const std::vector<std::string> & combo, double press_threshold);

/// Broadcasts a static parent->child TF from the EE-reference wrist midpoint.
/// The anchor is established automatically on the first valid EE reference (never
/// as an identity placeholder) and re-anchored thereafter by the ~/reset service,
/// a bimanual pinch (gloves), or a controller-button combo (joysticks).
class PoseResetNode : public rclcpp::Node
{
public:
  explicit PoseResetNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Track the wrist-midpoint anchor from the EE references; broadcast once on the
  /// first valid reference so teleop resolves without a manual trigger.
  void onEePoses(const geometry_msgs::msg::PoseArray::ConstSharedPtr & msg);

  /// ~/reset handler — re-anchor the transform to the current EE references.
  void onReset(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  /// Re-anchor while a simultaneous bimanual pinch is held (glove teleop).
  void onHand(const geometry_msgs::msg::PoseArray & msg);

  /// Re-anchor while the configured button combination is held (controller teleop).
  void onControllerData(const std_msgs::msg::ByteMultiArray & msg);

  /// Publish the current anchor if one has been computed; warn (throttled) and
  /// do nothing otherwise. Returns true iff a transform was broadcast.
  bool reAnchorIfReady();

  /// Broadcast the current parent->child transform to /tf_static.
  void publishStaticTransform();

  // All callbacks run on a single-threaded executor (the node's main() uses rclcpp::spin)
  // with no callback groups, so they are mutually exclusive. tf_mutex_ guards the
  // shared anchor state below as defensive cover should a MultiThreadedExecutor
  // ever be used.
  std::mutex tf_mutex_;
  Eigen::Isometry3d parent_pose_child_;  // guarded by tf_mutex_
  bool has_valid_anchor_ = false;        // guarded by tf_mutex_; set after first valid EE reference

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  std::string parent_frame_id_;
  std::string child_frame_id_;
  Eigen::Vector3d child_frame_offset_;

  // Glove pinch trigger (active when pinch_threshold_m_ > 0).
  double pinch_threshold_m_;

  // Controller-button trigger (active when reset_button_combo_ is non-empty).
  double press_threshold_;
  std::vector<std::string> reset_button_combo_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ee_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr hand_sub_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr controller_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace teleop
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_TELEOP__POSE_RESET_NODE_HPP_
