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

#ifndef ASL_FLIGHT2__MOCAP_RELAY_HPP_
#define ASL_FLIGHT2__MOCAP_RELAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

#include <memory>
#include <string>

namespace asl
{

class MocapRelay : public rclcpp::Node
{
public:
  /**
   * @brief constructor of motion capture relay base class
   *
   * @param node_name name of the node
   */
  explicit MocapRelay(const std::string & node_name = "mocap_relay");

protected:
  /**
   * @brief publish pose data to PX4 visual odometry channel
   *
   * @param msg pose message
   */
  void PublishPose(const geometry_msgs::msg::PoseStamped & msg) const;

private:
  const std::string world_frame_;
  const std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  const std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr odom_pub_;
};

}  // namespace asl

#endif  // ASL_FLIGHT2__MOCAP_RELAY_HPP_
