#include "asl_flight2/controller_base.hpp"

#include <chrono>

#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

enum ps4_axes_e {
  LEFT_LR = 0,
  LEFT_UD,
  L2,
  RIGHT_LR,
  RIGHT_UD,
  R2,
};

enum ps4_btn_e {
  X = 0,
  O,
  TRIANGLE,
  SQUARE,
  L1,
  R1,
};

const std::string mode = "body_rate";

class PS4Controller : public asl::ControllerBase {
 public:
  PS4Controller()
    : asl::ControllerBase("rc_ctrl"),
      joy_sub_(create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&PS4Controller::JoyCallback, this, std::placeholders::_1))) {}

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  double target_altitude;
  double target_yaw;

  void EnableControl() {
    if (mode == "velocity" && !ob_ctrl_mode_.velocity) {
      target_altitude = -this->world_t_body_.z();
      this->SetAltitude(target_altitude);
      this->SetVelocity({0.0f, 0.0f, NAN}, 0.0f);
      this->SetTrajCtrlMode(VELOCITY_ALTITUDE);
      RCLCPP_INFO(this->get_logger(), "velocity control enabled");
    } else if (mode == "attitude" && !ob_ctrl_mode_.attitude) {
      this->SetAttitude(Eigen::Quaterniond::Identity(), 0.0);
      this->SetAttitudeCtrlMode();
      RCLCPP_INFO(this->get_logger(), "attitude control enabled");
    } else if (mode == "body_rate" && !ob_ctrl_mode_.body_rate) {
      this->SetBodyRate(Eigen::Vector3d::Zero(), 0.0);
      this->SetBodyRateCtrlMode();
      RCLCPP_INFO(this->get_logger(), "body_rate control enabled");
    }
  }

  void DisableControl() {
    if ((mode == "velocity" && ob_ctrl_mode_.velocity) ||
        (mode == "attitude" && ob_ctrl_mode_.attitude) ||
        (mode == "body_rate" && ob_ctrl_mode_.body_rate)) {
      this->StopSetpointLoop();
      RCLCPP_INFO(this->get_logger(), "%s control disabled", mode.c_str());
    }
  }

  void UpdateControl(const sensor_msgs::msg::Joy::SharedPtr& msg) {
    if (mode == "velocity") {
      const Eigen::Vector3d velocity(
        2 * msg->axes[RIGHT_UD],
        -2 * msg->axes[RIGHT_LR],
        NAN
      );
      const double yaw_rate = -2 * msg->axes[LEFT_LR];
      this->SetVelocity(velocity, yaw_rate);

      target_altitude += .05 * msg->axes[LEFT_UD];
      this->SetAltitude(target_altitude);
    } else if (mode == "attitude") {
      const double yaw = -.5 * msg->axes[LEFT_LR];
      const double pitch = -.5 * msg->axes[RIGHT_UD];
      const double roll = -.5 * msg->axes[RIGHT_LR];
      const Eigen::Quaterniond attitude(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
      this->SetAttitude(attitude, (msg->axes[LEFT_UD] + 1) / 2);
    } else if (mode == "body_rate") {
      const double roll_rate = -5 * msg->axes[RIGHT_LR];
      const double pitch_rate = -5 * msg->axes[RIGHT_UD];
      const double yaw_rate = -2 * msg->axes[LEFT_LR];
      const double thrust = (msg->axes[LEFT_UD] + 1) / 2;
      this->SetBodyRate({roll_rate, pitch_rate, yaw_rate}, thrust);
    }
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // enable velocity control
    if (msg->buttons[O] && vehicle_ctrl_mode_.flag_armed) {
      EnableControl();
    }

    // disable velocity control
    if (msg->buttons[X] && ob_ctrl_mode_.velocity) {
      DisableControl();
    }

    if (msg->buttons[L1] && vehicle_status_.arming_state != vehicle_status_.ARMING_STATE_ARMED) {
      Arm();
    }

    if (msg->buttons[R1] && vehicle_status_.arming_state == vehicle_status_.ARMING_STATE_ARMED) {
      Disarm();
    }

    if (msg->buttons[SQUARE] && vehicle_status_.nav_state != vehicle_status_.NAVIGATION_STATE_AUTO_TAKEOFF) {
      Takeoff();
    }

    if (msg->buttons[TRIANGLE] && vehicle_status_.nav_state != vehicle_status_.NAVIGATION_STATE_AUTO_LAND) {
      Land();
    }

    // set target position
    if (vehicle_ctrl_mode_.flag_control_offboard_enabled) {
      UpdateControl(msg);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PS4Controller>());
  rclcpp::shutdown();
  return 0;
}
