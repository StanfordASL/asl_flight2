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

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "mocap_relay.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

using geometry_msgs::msg::PoseStamped;

class MocapRelayVRPN : public asl::MocapRelay
{
public:
  explicit MocapRelayVRPN(const std::string & node_name = "mocap_relay_vrpn")
  : asl::MocapRelay(node_name)
  {
    std::stringstream vrpn_channel_name;
    vrpn_channel_name <<
      "/vrpn_mocap/client_node/" <<
      this->declare_parameter<std::string>("vrpn_name", "asl_drone") <<
      "/pose";
    pose_sub_ = this->create_subscription<PoseStamped>(
      vrpn_channel_name.str(), 10,
      std::bind(&MocapRelayVRPN::PoseCallback, this, std::placeholders::_1));
  }

private:
  void PoseCallback(const PoseStamped::SharedPtr msg)
  {
    tf2::Transform world_nwu_T_body_nwu;
    tf2::fromMsg(msg->pose, world_nwu_T_body_nwu);
    const tf2::Transform nwu_T_neu(tf2::Quaternion(1, 0, 0, 0));
    const tf2::Transform world_nwu_T_body_neu = world_nwu_T_body_nwu * nwu_T_neu;

    PoseStamped pose_stamped = *msg;
    tf2::toMsg(world_nwu_T_body_neu, pose_stamped.pose);

    this->PublishPose(pose_stamped);
  }

  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MocapRelayVRPN>());

  rclcpp::shutdown();
}
