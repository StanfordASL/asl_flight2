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
#include <functional>
#include <memory>

#include "asl_flight2/controller_base.hpp"

using namespace std::chrono_literals;

class OffboardController : public asl::ControllerBase
{
public:
  OffboardController()
  : asl::ControllerBase("example_offboard_control")
  {
    // delay 5s mission start
    mission_timer_ = this->create_wall_timer(
      5s, std::bind(&OffboardController::MissionCallback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr mission_timer_;

  void MissionCallback()
  {
    this->Arm();
    while (!this->Armed()) {rclcpp::sleep_for(1ms);}

    this->Takeoff();
    rclcpp::sleep_for(10s);

    const double altitude = 1.5;
    constexpr double a = 1.0;
    this->SetTrajCtrlMode(POSITION_VELOCITY);
    this->SetPosition({0, 0, -altitude}, 0);
    this->SetVelocity({0, 0, 0});
    rclcpp::sleep_for(5s);
    double t = 0;
    auto start_time_ = this->get_clock()->now();
    while (t < 10.0) {
      const double sin_t = std::sin(t);
      const double cos_t = std::cos(t);
      const Eigen::Vector3d position(a * sin_t, a * sin_t * cos_t, -altitude);
      const Eigen::Vector3d velocity(a * cos_t, a * cos_t * cos_t - a * sin_t * sin_t, 0.0);
      const double yaw = std::atan2(velocity.y(), velocity.x());

      this->SetPosition(position, yaw);
      this->SetVelocity(velocity);

      t = this->get_clock()->now().seconds() - start_time_.seconds();
      rclcpp::sleep_for(1ms);
    }
    // this->StopSetpointLoop();
    rclcpp::sleep_for(1s);

    this->Land();

    mission_timer_ = nullptr;  // end mission
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<OffboardController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
