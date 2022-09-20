// Copyright 2022 Stanford ASL
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

#include "mocap_relay.hpp"

#include <memory>
#include <string>

namespace asl
{

using geometry_msgs::msg::PoseStamped;
using px4_msgs::msg::VehicleVisualOdometry;

MocapRelay::MocapRelay(const std::string & node_name)
: rclcpp::Node(node_name),
  world_frame_(this->declare_parameter("world_frame", "world_ned")),
  tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  odom_pub_(this->create_publisher<VehicleVisualOdometry>("fmu/vehicle_visual_odometry/in", 10)) {}

void MocapRelay::PublishPose(const geometry_msgs::msg::PoseStamped & msg) const
{
  geometry_msgs::msg::PoseStamped pose_world = msg;

  // transform frame if necessary
  if (msg.header.frame_id != world_frame_) {
    try {
      tf_buffer_->transform(msg, pose_world, world_frame_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not transform %s to %s: %s",
        msg.header.frame_id.c_str(), world_frame_.c_str(), ex.what());
      return;
    }
  }

  const rclcpp::Time timestamp_sample(msg.header.stamp);

  VehicleVisualOdometry odom{};
  odom.timestamp = this->now().nanoseconds() / 1e3;
  odom.timestamp_sample = timestamp_sample.nanoseconds() / 1e3;
  odom.pose_frame = VehicleVisualOdometry::POSE_FRAME_NED;

  odom.position[0] = pose_world.pose.position.x;
  odom.position[1] = pose_world.pose.position.y;
  odom.position[2] = pose_world.pose.position.z;
  odom.q[0] = pose_world.pose.orientation.w;
  odom.q[1] = pose_world.pose.orientation.x;
  odom.q[2] = pose_world.pose.orientation.y;
  odom.q[3] = pose_world.pose.orientation.z;

  odom.position_variance[0] = NAN;
  odom.position_variance[1] = NAN;
  odom.position_variance[2] = NAN;
  odom.orientation_variance[0] = NAN;
  odom.orientation_variance[1] = NAN;
  odom.orientation_variance[2] = NAN;

  odom_pub_->publish(odom);
}

}  // namespace asl
