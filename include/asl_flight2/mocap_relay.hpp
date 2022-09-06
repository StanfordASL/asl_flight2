#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

namespace asl {

class MocapRelay : public rclcpp::Node {
 public:
  explicit MocapRelay(const std::string& node_name = "mocap_relay");

 protected:
  void PublishPose(const geometry_msgs::msg::PoseStamped& msg);

 private:
  const std::string world_frame_;
  const std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  const std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr odom_pub_;
};

} // namespace asl
