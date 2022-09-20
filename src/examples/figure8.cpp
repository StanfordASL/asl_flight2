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

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "asl_flight2/controller_base.hpp"
#include "asl_flight2/ps4_def.hpp"

#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class Figure8Controller : public asl::ControllerBase
{
public:
  Figure8Controller()
  : asl::ControllerBase("example_figure8"),
    joy_sub_(create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        10,
        std::bind(&Figure8Controller::JoyCallback, this, std::placeholders::_1),
        parallel_sub_options_)) {}

private:
  rclcpp::Time start_time_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void SetPositionVelocity(const double & altitude = 1.5)
  {
    const double t = this->get_clock()->now().seconds() - start_time_.seconds();
    constexpr double a = 1.0;
    const double sin_t = std::sin(t);
    const double cos_t = std::cos(t);
    const Eigen::Vector3d position(a * sin_t, a * sin_t * cos_t, -altitude);
    const Eigen::Vector3d velocity(a * cos_t, a * cos_t * cos_t - a * sin_t * sin_t, 0.0);
    const double yaw = std::atan2(velocity.y(), velocity.x());

    this->SetPosition(position, yaw);
    this->SetVelocity(velocity);
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons[O] && !this->SetpointRunning() && this->IsAirborne()) {
      start_time_ = this->get_clock()->now();
      this->SetTrajCtrlMode(POSITION_VELOCITY);
    }

    if (msg->buttons[X]) {
      this->StopSetpointLoop();
    }

    if (msg->buttons[L1]) {
      this->Arm();
    }

    if (msg->buttons[R1]) {
      this->Disarm();
    }

    if (msg->buttons[SQUARE]) {
      this->Takeoff();
    }

    if (msg->buttons[TRIANGLE]) {
      this->Land();
    }

    this->SetPositionVelocity();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<Figure8Controller>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
