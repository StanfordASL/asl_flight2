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

#include "asl_flight2/controller_base.hpp"

#include <chrono>

using namespace std::chrono_literals;

class TakeoffController : public asl::ControllerBase {
 public:
  TakeoffController() : ControllerBase("takeoff_example") {
    rclcpp::sleep_for(5s);

    this->Arm();
    rclcpp::sleep_for(1s);

    this->Takeoff();
    rclcpp::sleep_for(10s);

    this->Land();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffController>());
  rclcpp::shutdown();
  return 0;
}
