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

class TakeoffController : public asl::ControllerBase
{
public:
  TakeoffController()
  : asl::ControllerBase("example_takeoff")
  {
    // delay 5s mission start
    mission_timer_ = this->create_wall_timer(
      5s, std::bind(&TakeoffController::MissionCallback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr mission_timer_;

  void MissionCallback()
  {
    this->Arm();
    while (!this->Armed()) {rclcpp::sleep_for(1ms);}

    this->Takeoff();
    rclcpp::sleep_for(10s);

    this->Land();

    mission_timer_ = nullptr;  // end mission
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<TakeoffController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
