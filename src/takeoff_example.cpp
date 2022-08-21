#include "asl_flight2/controller_base.h"

#include <chrono>

using namespace std::chrono_literals;

class TakeoffController : public asl::ControllerBase {
 public:
  TakeoffController() : ControllerBase("takeoff_example") {
    ctrl_timer_ = this->create_wall_timer(1ms, [this]() {
      RCLCPP_INFO(this->get_logger(), "hi");
      SetPosition({0., 0., -1.}, 0.);
    });

    rclcpp::sleep_for(5s);
    RCLCPP_WARN(this->get_logger(), "Waked Up");

    EnableOffboardCtrl();
    Arm();
  }

 private:
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffController>());
  rclcpp::shutdown();
  return 0;
}
