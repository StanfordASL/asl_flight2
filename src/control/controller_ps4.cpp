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
#include <memory>
#include <string>

#include "asl_flight2/controller_base.hpp"

#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

enum ps4_axes_e
{
  LEFT_LR = 0,
  LEFT_UD,
  L2,
  RIGHT_LR,
  RIGHT_UD,
  R2,
};

enum ps4_btn_e
{
  X = 0,
  O,
  TRIANGLE,
  SQUARE,
  L1,
  R1,
};

class PS4Controller : public asl::ControllerBase
{
public:
  PS4Controller()
  : asl::ControllerBase("rc_ctrl"),
    joy_sub_(create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&PS4Controller::JoyCallback, this, std::placeholders::_1))),
    mode_(this->declare_parameter("mode", "velocity")) {}

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  double target_altitude_;
  const std::string mode_;

  void EnableControl()
  {
    if (mode_ == "velocity") {
      target_altitude_ = -this->GetVehicleState().world_t_body.z();
      this->SetAltitude(target_altitude_);
      this->SetVelocity({0.0f, 0.0f, NAN}, 0.0f);
      this->SetTrajCtrlMode(VELOCITY_ALTITUDE);
    } else if (mode_ == "attitude") {
      this->SetAttitude(Eigen::Quaterniond::Identity(), 0.0);
      this->SetAttitudeCtrlMode();
    } else if (mode_ == "body_rate") {
      this->SetBodyRate(Eigen::Vector3d::Zero(), 0.0);
      this->SetBodyRateCtrlMode();
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Enable control failed: mode %s unsupported", mode_.c_str());
    }
  }

  void DisableControl()
  {
    if ((mode_ == "velocity" && ob_ctrl_mode_.velocity) ||
      (mode_ == "attitude" && ob_ctrl_mode_.attitude) ||
      (mode_ == "body_rate" && ob_ctrl_mode_.body_rate))
    {
      this->StopSetpointLoop();
      RCLCPP_INFO(this->get_logger(), "%s control disabled", mode_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Control not enabled");
    }
  }

  void UpdateControl(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    if (mode_ == "velocity") {
      const Eigen::Vector3d velocity(
        2 * msg->axes[RIGHT_UD],
        -2 * msg->axes[RIGHT_LR],
        NAN
      );
      const double yaw_rate = -2 * msg->axes[LEFT_LR];
      this->SetVelocity(velocity, yaw_rate);

      target_altitude_ += .05 * msg->axes[LEFT_UD];
      this->SetAltitude(target_altitude_);
    } else if (mode_ == "attitude") {
      const double yaw = -.5 * msg->axes[LEFT_LR];
      const double pitch = -.5 * msg->axes[RIGHT_UD];
      const double roll = -.5 * msg->axes[RIGHT_LR];
      const Eigen::Quaterniond attitude(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
      this->SetAttitude(attitude, (msg->axes[LEFT_UD] + 1) / 2);
    } else if (mode_ == "body_rate") {
      const double roll_rate = -5 * msg->axes[RIGHT_LR];
      const double pitch_rate = -5 * msg->axes[RIGHT_UD];
      const double yaw_rate = -2 * msg->axes[LEFT_LR];
      const double thrust = (msg->axes[LEFT_UD] + 1) / 2;
      this->SetBodyRate({roll_rate, pitch_rate, yaw_rate}, thrust);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Should not reach here");
    }
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons[O] && this->IsAirborne()) {
      this->EnableControl();
    }

    if (msg->buttons[X]) {
      this->DisableControl();
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

    // set target position
    if (this->OffboardEnabled()) {
      UpdateControl(msg);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(std::make_shared<PS4Controller>());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
