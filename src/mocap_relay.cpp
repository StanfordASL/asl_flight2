#include "asl_flight2/mocap_relay.hpp"

namespace asl {

using geometry_msgs::msg::PoseStamped;
using px4_msgs::msg::VehicleVisualOdometry;

MocapRelay::MocapRelay(const std::string& node_name)
  : rclcpp::Node(node_name),
    world_frame_(this->declare_parameter("world_frame", "world_ned")),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    odom_pub_(this->create_publisher<VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in", 10)) {}

void MocapRelay::PublishPose(const geometry_msgs::msg::PoseStamped& msg) {
  geometry_msgs::msg::PoseStamped pose_world = msg;

  // transform frame if necessary
  if (msg.header.frame_id != world_frame_) {
    try {
      tf_buffer_->transform(msg, pose_world, world_frame_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
        msg.header.frame_id.c_str(), world_frame_.c_str(), ex.what());
      return ;
    }
  }

  rclcpp::Time timestamp_sample = msg.header.stamp;

  VehicleVisualOdometry odom{};
  odom.timestamp = this->get_clock()->now().nanoseconds() / 1e3;
  odom.timestamp_sample = timestamp_sample.nanoseconds() / 1e3;
  odom.local_frame = VehicleVisualOdometry::LOCAL_FRAME_NED;

  odom.x = pose_world.pose.position.x;
  odom.y = pose_world.pose.position.y;
  odom.z = pose_world.pose.position.z;
  odom.q[0] = pose_world.pose.orientation.w;
  odom.q[1] = pose_world.pose.orientation.x;
  odom.q[2] = pose_world.pose.orientation.y;
  odom.q[3] = pose_world.pose.orientation.z;

  odom.pose_covariance[VehicleVisualOdometry::COVARIANCE_MATRIX_X_VARIANCE] = NAN;
  odom.pose_covariance[VehicleVisualOdometry::COVARIANCE_MATRIX_ROLL_VARIANCE] = NAN;

  odom_pub_->publish(odom);
}

} // namespace asl
