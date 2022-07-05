#include "asl_flight2/controller_base.hpp"

#include <chrono>
#include <functional>

#include <px4_ros_com/frame_transforms.h>
#include <tf2_eigen/tf2_eigen.h>

namespace asl {

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

const Eigen::Quaterniond nwu_R_ned(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

// TODO: understand ROS2 QoS profiles better
ControllerBase::ControllerBase(const std::string& node_name, const size_t qos_history_depth)
    : rclcpp::Node(node_name),
      sub_odom_(create_subscription<VehicleOdometry>(
        "/fmu/vehicle_odometry/out",
        qos_history_depth,
        std::bind(&ControllerBase::VehicleOdometryCallback, this, _1)
      )),
      sub_timesync_(create_subscription<Timesync>(
        "/fmu/timesync/out",
        qos_history_depth,
        std::bind(&ControllerBase::TimeSyncCallback, this, _1)
      )),
      pose_pub_(create_publisher<PoseStamped>("pose", qos_history_depth)) {
}

void ControllerBase::VehicleOdometryCallback(const VehicleOdometry::SharedPtr msg) const {
  // position
  const Eigen::Vector3d world_t_body_ned(msg->x, msg->y, msg->z);
  world_t_body_ = nwu_R_ned * world_t_body_ned;

  // orientation
  const Eigen::Quaterniond world_R_body_ned =
    px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
  world_R_body_ = nwu_R_ned * world_R_body_ned.normalized() * nwu_R_ned.conjugate();

  // velocity
  const Eigen::Vector3d v_body_ned(msg->vx, msg->vy, msg->vz);
  v_body_ = nwu_R_ned * v_body_ned;
  v_world_ = world_R_body_ * v_body_;

  // anguler velocity
  const Eigen::Vector3d w_body_ned(msg->rollspeed, msg->pitchspeed, msg->yawspeed);
  w_body_ = nwu_R_ned * w_body_ned;
  w_world_ = world_R_body_ * w_body_;

  // re-publish pose for rviz visualization
  PoseStamped pose_msg{};
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = rclcpp::Clock().now();
  pose_msg.pose.position = tf2::toMsg(world_t_body_);
  pose_msg.pose.orientation = tf2::toMsg(world_R_body_);
  pose_pub_->publish(pose_msg);
}

void ControllerBase::TimeSyncCallback(const Timesync::SharedPtr msg) const {
  const uint64_t time_sync = msg->timestamp;
  const uint64_t time_now = rclcpp::Clock().now().nanoseconds() / 1e3;
  timestamp_synced_.store(msg->timestamp);
  RCLCPP_INFO(this->get_logger(), "%lu %lu %ld", time_sync, time_now, (int64_t)time_now - time_sync);
}

} // namespace asl

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<asl::ControllerBase>("controller_base"));
    rclcpp::shutdown();
    return 0;
}
