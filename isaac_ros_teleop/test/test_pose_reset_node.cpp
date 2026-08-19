// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "gtest/gtest.h"
#include "msgpack.h"

#include "isaac_ros_teleop/pose_reset_node.hpp"

namespace teleop = nvidia::isaac_ros::teleop;

namespace
{

geometry_msgs::msg::Pose MakePose(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;  // identity orientation
  return pose;
}

using ByteT = std_msgs::msg::ByteMultiArray::_data_type::value_type;

// Pack {key: bool/number} pairs into a ByteMultiArray, optionally prefixed by
// `offset` filler bytes with layout.data_offset set accordingly.
std_msgs::msg::ByteMultiArray PackComboMsg(
  const std::vector<std::pair<std::string, double>> & fields, size_t offset = 0)
{
  msgpack_sbuffer sbuf;
  msgpack_sbuffer_init(&sbuf);
  msgpack_packer pk;
  msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);
  msgpack_pack_map(&pk, fields.size());
  for (const auto & [key, value] : fields) {
    msgpack_pack_str(&pk, key.size());
    msgpack_pack_str_body(&pk, key.data(), key.size());
    msgpack_pack_double(&pk, value);
  }

  std_msgs::msg::ByteMultiArray msg;
  msg.layout.data_offset = static_cast<uint32_t>(offset);
  for (size_t i = 0; i < offset; ++i) {
    msg.data.push_back(static_cast<ByteT>(0));
  }
  for (size_t i = 0; i < sbuf.size; ++i) {
    msg.data.push_back(static_cast<ByteT>(static_cast<unsigned char>(sbuf.data[i])));
  }
  msgpack_sbuffer_destroy(&sbuf);
  return msg;
}

const std::vector<std::string> kCombo{"left_secondary_click", "right_secondary_click"};

}  // namespace

// ----------------------------- IsZeroPose ---------------------------------

TEST(IsZeroPose, IdentityAtOriginIsZero)
{
  EXPECT_TRUE(teleop::IsZeroPose(MakePose(0.0, 0.0, 0.0)));
}

TEST(IsZeroPose, OffsetPositionIsNotZero)
{
  EXPECT_FALSE(teleop::IsZeroPose(MakePose(0.5, 0.0, 0.0)));
}

TEST(IsZeroPose, RotatedOrientationIsNotZero)
{
  geometry_msgs::msg::Pose pose = MakePose(0.0, 0.0, 0.0);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  EXPECT_FALSE(teleop::IsZeroPose(pose));
}

// ------------------------- ComputeParentPoseChild -------------------------

TEST(ComputeParentPoseChild, AxisAlignedFrameAndOffset)
{
  // Left at +Y, right at -Y (ROS FLU): heading is +X, frame is identity rotation.
  // With identity rotation the child-frame offset maps directly onto the origin.
  const auto result = teleop::ComputeParentPoseChild(
    MakePose(0.0, 0.2), MakePose(0.0, -0.2), Eigen::Vector3d(-0.25, 0.1, 0.05));
  ASSERT_TRUE(result.has_value());

  const Eigen::Matrix3d r = result->rotation();
  // Right-handed orthonormal rotation.
  EXPECT_NEAR(r.determinant(), 1.0, 1e-9);
  EXPECT_TRUE((r * r.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  EXPECT_TRUE(r.isApprox(Eigen::Matrix3d::Identity(), 1e-9));

  // Origin = XY midpoint (0,0,0) plus the full [x, y, z] offset in child frame.
  EXPECT_NEAR(result->translation().x(), -0.25, 1e-9);
  EXPECT_NEAR(result->translation().y(), 0.1, 1e-9);
  EXPECT_NEAR(result->translation().z(), 0.05, 1e-9);
}

TEST(ComputeParentPoseChild, RotatedReferencesStayOrthonormal)
{
  // References offset along X as well -> non-identity heading.
  const auto result = teleop::ComputeParentPoseChild(
    MakePose(0.3, 0.2), MakePose(0.1, -0.2), Eigen::Vector3d::Zero());
  ASSERT_TRUE(result.has_value());
  const Eigen::Matrix3d r = result->rotation();
  EXPECT_NEAR(r.determinant(), 1.0, 1e-9);
  EXPECT_TRUE((r * r.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  // Z stays world-up.
  EXPECT_TRUE(r.col(2).isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
  // Origin = XY midpoint (no offset).
  EXPECT_NEAR(result->translation().x(), 0.2, 1e-9);
  EXPECT_NEAR(result->translation().y(), 0.0, 1e-9);
}

TEST(ComputeParentPoseChild, CoincidentReferencesAreDegenerate)
{
  const auto result = teleop::ComputeParentPoseChild(
    MakePose(0.1, 0.1), MakePose(0.1, 0.1), Eigen::Vector3d::Zero());
  EXPECT_FALSE(result.has_value());
}

// --------------------------- PositionDistance -----------------------------

TEST(PositionDistance, ComputesEuclideanDistance)
{
  geometry_msgs::msg::PoseArray poses;
  poses.poses.resize(48);
  poses.poses[teleop::kLeftThumbTipIdx] = MakePose(0.0, 0.0, 0.0);
  poses.poses[teleop::kLeftLittleTipIdx] = MakePose(0.03, 0.04, 0.0);  // 0.05 away
  const auto d = teleop::PositionDistance(
    poses, teleop::kLeftThumbTipIdx, teleop::kLeftLittleTipIdx);
  ASSERT_TRUE(d.has_value());
  EXPECT_NEAR(d.value(), 0.05f, 1e-5f);
}

TEST(PositionDistance, OutOfRangeReturnsNullopt)
{
  geometry_msgs::msg::PoseArray poses;
  poses.poses.resize(5);
  EXPECT_FALSE(teleop::PositionDistance(poses, 0, 47).has_value());
}

// --------------------------- EvaluateResetCombo ---------------------------

TEST(EvaluateResetCombo, AllPressed)
{
  const auto msg = PackComboMsg(
    {{"left_secondary_click", 1.0}, {"right_secondary_click", 1.0}});
  const auto state = teleop::EvaluateResetCombo(msg, kCombo, 0.5);
  EXPECT_TRUE(state.decoded);
  EXPECT_TRUE(state.any_combo_key_seen);
  EXPECT_TRUE(state.all_pressed);
}

TEST(EvaluateResetCombo, OneBelowThresholdIsNotPressed)
{
  const auto msg = PackComboMsg(
    {{"left_secondary_click", 1.0}, {"right_secondary_click", 0.0}});
  const auto state = teleop::EvaluateResetCombo(msg, kCombo, 0.5);
  EXPECT_TRUE(state.decoded);
  EXPECT_FALSE(state.all_pressed);
}

TEST(EvaluateResetCombo, MissingKeyIsNotPressed)
{
  const auto msg = PackComboMsg({{"left_secondary_click", 1.0}});
  const auto state = teleop::EvaluateResetCombo(msg, kCombo, 0.5);
  EXPECT_TRUE(state.decoded);
  EXPECT_TRUE(state.any_combo_key_seen);
  EXPECT_FALSE(state.all_pressed);
}

TEST(EvaluateResetCombo, UnknownKeysOnlyFlagsNoComboKeySeen)
{
  const auto msg = PackComboMsg({{"some_other_button", 1.0}});
  const auto state = teleop::EvaluateResetCombo(msg, kCombo, 0.5);
  EXPECT_TRUE(state.decoded);
  EXPECT_FALSE(state.any_combo_key_seen);
  EXPECT_FALSE(state.all_pressed);
}

TEST(EvaluateResetCombo, EmptyDataDoesNotDecode)
{
  std_msgs::msg::ByteMultiArray msg;
  const auto state = teleop::EvaluateResetCombo(msg, kCombo, 0.5);
  EXPECT_FALSE(state.decoded);
  EXPECT_FALSE(state.all_pressed);
}

TEST(EvaluateResetCombo, HonorsDataOffset)
{
  const auto msg = PackComboMsg(
    {{"left_secondary_click", 1.0}, {"right_secondary_click", 1.0}}, /*offset=*/3);
  const auto state = teleop::EvaluateResetCombo(msg, kCombo, 0.5);
  EXPECT_TRUE(state.decoded);
  EXPECT_TRUE(state.all_pressed);
}

TEST(EvaluateResetCombo, EmptyComboIsDisabled)
{
  const auto msg = PackComboMsg({{"left_secondary_click", 1.0}});
  const auto state = teleop::EvaluateResetCombo(msg, {}, 0.5);
  EXPECT_FALSE(state.all_pressed);
}
