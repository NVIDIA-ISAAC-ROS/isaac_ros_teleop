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

#include "isaac_ros_teleop/pose_reset_node.hpp"

#include <msgpack.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace
{

// Numeric value of a msgpack scalar (float/int/bool), or nullopt for other types.
std::optional<double> AsNumber(const msgpack_object & obj)
{
  switch (obj.type) {
    case MSGPACK_OBJECT_FLOAT32:
    case MSGPACK_OBJECT_FLOAT64:
      return obj.via.f64;
    case MSGPACK_OBJECT_POSITIVE_INTEGER:
      return static_cast<double>(obj.via.u64);
    case MSGPACK_OBJECT_NEGATIVE_INTEGER:
      return static_cast<double>(obj.via.i64);
    case MSGPACK_OBJECT_BOOLEAN:
      return obj.via.boolean ? 1.0 : 0.0;
    default:
      return std::nullopt;
  }
}

}  // namespace

namespace nvidia
{
namespace isaac_ros
{
namespace teleop
{

bool IsZeroPose(const geometry_msgs::msg::Pose & pose, double pos_tol, double rot_tol)
{
  const auto pos_norm = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z).norm();
  const auto q = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  const auto rot_angle = Eigen::AngleAxisd(q.normalized()).angle();
  return pos_norm < pos_tol && std::abs(rot_angle) < rot_tol;
}

std::optional<Eigen::Isometry3d> ComputeParentPoseChild(
  const geometry_msgs::msg::Pose & left, const geometry_msgs::msg::Pose & right,
  const Eigen::Vector3d & child_frame_offset)
{
  const double midpoint_x = 0.5 * (left.position.x + right.position.x);
  const double midpoint_y = 0.5 * (left.position.y + right.position.y);

  // `_pc` suffix = child-frame axis expressed in the parent frame (column of parent_pose_child).
  const Eigen::Vector3d z_axis_pc(0, 0, 1);
  const Eigen::Vector3d y_axis_raw(
    left.position.x - right.position.x,
    left.position.y - right.position.y,
    0);
  // Degenerate when both EE references coincide in XY — heading is undefined.
  if (y_axis_raw.norm() < kMinReferenceSeparationM) {
    return std::nullopt;
  }
  const Eigen::Vector3d y_axis_pc = y_axis_raw.normalized();
  const Eigen::Vector3d x_axis_pc = y_axis_pc.cross(z_axis_pc);

  Eigen::Matrix4d parent_pose_child = Eigen::Matrix4d::Identity();
  parent_pose_child.block<3, 1>(0, 0) = x_axis_pc;
  parent_pose_child.block<3, 1>(0, 1) = y_axis_pc;
  parent_pose_child.block<3, 1>(0, 2) = z_axis_pc;
  parent_pose_child.block<3, 1>(0, 3) = Eigen::Vector3d(midpoint_x, midpoint_y, 0.0);

  // Static translational offset expressed in the child frame.
  Eigen::Matrix4d child_pose_offset = Eigen::Matrix4d::Identity();
  child_pose_offset.block<3, 1>(0, 3) = child_frame_offset;

  Eigen::Isometry3d result;
  result.matrix() = parent_pose_child * child_pose_offset;
  if (!result.matrix().allFinite()) {
    return std::nullopt;
  }
  return result;
}

std::optional<float> PositionDistance(
  const geometry_msgs::msg::PoseArray & poses, int idx_a, int idx_b)
{
  const int max_idx = std::max(idx_a, idx_b);
  if (idx_a < 0 || idx_b < 0 || poses.poses.size() <= static_cast<size_t>(max_idx)) {
    return std::nullopt;
  }
  const auto & a = poses.poses[idx_a].position;
  const auto & b = poses.poses[idx_b].position;
  const Eigen::Vector3d va(a.x, a.y, a.z);
  const Eigen::Vector3d vb(b.x, b.y, b.z);
  return static_cast<float>((va - vb).norm());
}

ResetComboState EvaluateResetCombo(
  const std_msgs::msg::ByteMultiArray & msg,
  const std::vector<std::string> & combo, double press_threshold)
{
  ResetComboState state;
  if (combo.empty()) {
    return state;  // trigger disabled
  }

  // MultiArray payloads conventionally start at layout.data_offset.
  const size_t offset_bytes = msg.layout.data_offset;
  if (offset_bytes >= msg.data.size()) {
    return state;  // nothing to decode
  }

  // pressed[b] / seen[b] track each configured combo key independently.
  std::vector<bool> pressed(combo.size(), false);
  std::vector<bool> seen(combo.size(), false);

  msgpack_unpacked und;
  msgpack_unpacked_init(&und);
  size_t offset = 0;
  const msgpack_unpack_return ret = msgpack_unpack_next(
    &und,
    reinterpret_cast<const char *>(msg.data.data()) + offset_bytes,
    msg.data.size() - offset_bytes, &offset);

  if (ret == MSGPACK_UNPACK_SUCCESS && und.data.type == MSGPACK_OBJECT_MAP) {
    state.decoded = true;
    const msgpack_object_map & map = und.data.via.map;
    for (uint32_t i = 0; i < map.size; ++i) {
      const msgpack_object_kv & kv = map.ptr[i];
      if (kv.key.type != MSGPACK_OBJECT_STR) {
        continue;
      }
      const std::string key(kv.key.via.str.ptr, kv.key.via.str.size);
      const std::optional<double> value = AsNumber(kv.val);
      for (size_t b = 0; b < combo.size(); ++b) {
        if (key == combo[b]) {
          seen[b] = true;
          if (value) {
            pressed[b] = *value >= press_threshold;
          }
        }
      }
    }
  }
  msgpack_unpacked_destroy(&und);

  state.any_combo_key_seen =
    std::any_of(seen.begin(), seen.end(), [](bool s) {return s;});
  state.all_pressed = state.decoded &&
    std::all_of(pressed.begin(), pressed.end(), [](bool p) {return p;});
  return state;
}

PoseResetNode::PoseResetNode(const rclcpp::NodeOptions & options)
: Node("pose_reset_node", options),
  parent_pose_child_(Eigen::Isometry3d::Identity()),
  static_tf_broadcaster_(this),
  parent_frame_id_(declare_parameter<std::string>("parent_frame_id", "world")),
  child_frame_id_(declare_parameter<std::string>("child_frame_id", "base_link")),
  pinch_threshold_m_(declare_parameter<double>("pinch_threshold_m", 0.025)),
  press_threshold_(declare_parameter<double>("press_threshold", 0.5)),
  reset_button_combo_(declare_parameter<std::vector<std::string>>(
      "reset_button_combo",
      std::vector<std::string>{"left_secondary_click", "right_secondary_click"}))
{
  // Validate parameters: a non-positive press_threshold makes a released button
  // (value 0.0) read as pressed, which would re-anchor continuously.
  if (!(press_threshold_ > 0.0)) {
    RCLCPP_WARN(
      get_logger(),
      "press_threshold (%.3f) must be > 0 — a released button would read as pressed. "
      "Falling back to 0.5.", press_threshold_);
    press_threshold_ = 0.5;
  }
  if (!std::isfinite(pinch_threshold_m_)) {
    RCLCPP_WARN(get_logger(), "pinch_threshold_m is not finite; disabling the pinch trigger.");
    pinch_threshold_m_ = 0.0;
  }

  const auto offset = declare_parameter<std::vector<double>>(
    "child_frame_offset", std::vector<double>{0.0, 0.0, 0.0});
  if (offset.size() != 3) {
    RCLCPP_WARN(
      get_logger(),
      "child_frame_offset must have 3 elements [x, y, z]; got %zu — using [0, 0, 0].",
      offset.size());
    child_frame_offset_ = Eigen::Vector3d::Zero();
  } else {
    child_frame_offset_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);
  }

  ee_poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "/xr_teleop/ee_poses",
    rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
      onEePoses(msg);
    });

  // Pinch and button reset triggers are optional; enabled by their params below.
  if (pinch_threshold_m_ > 0.0) {
    hand_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/xr_teleop/hand",
      rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
        onHand(*msg);
      });
  }

  if (!reset_button_combo_.empty()) {
    controller_sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/xr_teleop/controller_data",
      rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::ByteMultiArray::ConstSharedPtr msg) {
        onControllerData(*msg);
      });
  }

  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      onReset(request, response);
    });

  if (pinch_threshold_m_ <= 0.0 && reset_button_combo_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Both the pinch and button reset triggers are disabled; only the ~/reset "
      "service can re-anchor.");
  }

  // Deliberately do NOT broadcast here. The anchor is auto-established on the
  // first valid EE reference (see onEePoses), so the node never advertises an
  // identity placeholder that the IK controller would resolve against.
  RCLCPP_INFO(
    get_logger(),
    "pose_reset_node ready; will auto-anchor on the first valid EE reference.");
}

void PoseResetNode::onEePoses(const geometry_msgs::msg::PoseArray::ConstSharedPtr & msg)
{
  if (msg->poses.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Expected 2 reference poses (left + right), got %zu — skipping update.",
      msg->poses.size());
    return;
  }

  if (IsZeroPose(msg->poses[0]) || IsZeroPose(msg->poses[1])) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Received zero-valued reference pose (uninitialized upstream) — skipping update.");
    return;
  }

  const std::optional<Eigen::Isometry3d> parent_pose_child =
    ComputeParentPoseChild(msg->poses[0], msg->poses[1], child_frame_offset_);
  if (!parent_pose_child) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "EE references coincide or produced a non-finite transform — skipping update.");
    return;
  }

  bool first_anchor = false;
  {
    const std::lock_guard<std::mutex> lock(tf_mutex_);
    parent_pose_child_ = *parent_pose_child;
    first_anchor = !has_valid_anchor_;
    has_valid_anchor_ = true;
  }

  // Auto-anchor on the first valid EE reference so teleop resolves immediately
  // (no manual trigger needed at startup). The latched /tf_static keeps it live
  // for late subscribers. Triggers (~/reset, pinch, button) re-anchor thereafter.
  if (first_anchor) {
    publishStaticTransform();
    RCLCPP_INFO(
      get_logger(), "Auto-anchored %s->%s from the first valid EE reference.",
      parent_frame_id_.c_str(), child_frame_id_.c_str());
  }
}

void PoseResetNode::onReset(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (reAnchorIfReady()) {
    response->success = true;
    response->message = "Re-anchored world_teleop->pelvis to the current EE references.";
  } else {
    response->success = false;
    response->message = "No valid EE reference received yet; nothing to anchor to.";
  }
}

void PoseResetNode::onHand(const geometry_msgs::msg::PoseArray & msg)
{
  const std::optional<float> left = PositionDistance(msg, kLeftThumbTipIdx, kLeftLittleTipIdx);
  const std::optional<float> right = PositionDistance(msg, kRightThumbTipIdx, kRightLittleTipIdx);
  const bool pinched =
    left.has_value() && right.has_value() &&
    left.value() < pinch_threshold_m_ && right.value() < pinch_threshold_m_;

  // Deliberate continuous re-anchor while held; latched /tf_static keeps the last pose.
  if (pinched && reAnchorIfReady()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "bimanual pinch held (l=%.1f cm, r=%.1f cm) — re-anchoring (continuous, throttled).",
      left.value() * 100.0f, right.value() * 100.0f);
  }
}

void PoseResetNode::onControllerData(const std_msgs::msg::ByteMultiArray & msg)
{
  const ResetComboState state =
    EvaluateResetCombo(msg, reset_button_combo_, press_threshold_);

  if (!state.decoded) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "Could not decode /xr_teleop/controller_data as a msgpack map; ignoring.");
  } else if (!state.any_combo_key_seen) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "None of the configured reset_button_combo keys were present in "
      "/xr_teleop/controller_data; check the field names against the publisher.");
  }

  // Same deliberate continuous re-anchor as the pinch trigger (see onHand).
  if (state.all_pressed && reAnchorIfReady()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "reset button combo held — re-anchoring continuously (log throttled).");
  }
}

bool PoseResetNode::reAnchorIfReady()
{
  {
    const std::lock_guard<std::mutex> lock(tf_mutex_);
    if (!has_valid_anchor_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Re-anchor requested but no valid EE reference has been received yet; ignoring.");
      return false;
    }
  }
  publishStaticTransform();
  return true;
}

void PoseResetNode::publishStaticTransform()
{
  geometry_msgs::msg::TransformStamped tf;
  {
    const std::lock_guard<std::mutex> lock(tf_mutex_);
    tf = tf2::eigenToTransform(parent_pose_child_);
  }
  // Static transforms are time-independent in tf2; stamp with the broadcast time
  // (rather than the source EE stamp) so the entry is valid regardless of upstream
  // stamp hygiene.
  tf.header.stamp = now();
  tf.header.frame_id = parent_frame_id_;
  tf.child_frame_id = child_frame_id_;
  static_tf_broadcaster_.sendTransform(tf);
}

}  // namespace teleop
}  // namespace isaac_ros
}  // namespace nvidia
