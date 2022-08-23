#include "asl_flight2/controller_base.h"

#include <chrono>
#include <memory>

#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("takeoff_command");
  auto cmd_pub = node->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/vehicle_command/in", 10);

  px4_msgs::msg::VehicleCommand msg{};
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  // offboard enabled
  msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
  msg.command = msg.VEHICLE_CMD_DO_SET_MODE;
  msg.param1 = 1;
  msg.param2 = 6;
  cmd_pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "offboard control enabled");
  rclcpp::sleep_for(1s);

  // arm
  msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
  msg.command = msg.VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg.param1 = msg.ARMING_ACTION_ARM;
  cmd_pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "armed");
  rclcpp::sleep_for(2s);

  // takeoff
  msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
  msg.command = msg.VEHICLE_CMD_NAV_TAKEOFF;
  msg.param1 = NAN;
  msg.param2 = NAN;
  msg.param3 = NAN;
  msg.param4 = NAN;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = 1.0;
  cmd_pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "takeoff");
  rclcpp::sleep_for(20s);

  // land
  msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
  msg.command = msg.VEHICLE_CMD_NAV_LAND;
  msg.param1 = NAN;
  msg.param2 = NAN;
  msg.param3 = NAN;
  msg.param4 = NAN;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = 0.0;
  cmd_pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "land");
  rclcpp::sleep_for(3s);

  rclcpp::shutdown();
  return 0;
}
