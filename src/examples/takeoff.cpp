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
