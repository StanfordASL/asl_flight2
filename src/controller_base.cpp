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

// TODO: understand ROS2 QoS profiles better
ControllerBase::ControllerBase(const std::string& node_name, const size_t qos_history_depth)
    : rclcpp::Node(node_name),
      sub_timesync_(create_subscription<Timesync>(
        "/fmu/timesync/out",
        qos_history_depth,
        std::bind(&ControllerBase::TimeSyncCallback, this, _1)
      )),
      sub_ctrl_mode_(create_subscription<VehicleControlMode>(
        "/fmu/vehicle_control_mode/out",
        qos_history_depth,
        [this](const VehicleControlMode::SharedPtr msg) { vehicle_ctrl_mode_ = *msg; }
      )),
      sub_odom_(create_subscription<VehicleOdometry>(
        "/fmu/vehicle_odometry/out",
        qos_history_depth,
        std::bind(&ControllerBase::VehicleOdometryCallback, this, _1)
      )),
      sub_status_(create_subscription<VehicleStatus>(
        "/fmu/vehicle_status/out",
        qos_history_depth,
        [this](const VehicleStatus::SharedPtr msg) { vehicle_status_ = *msg; }
      )),
      pose_pub_(create_publisher<PoseStamped>("pose", qos_history_depth)),
      offboard_mode_pub_(create_publisher<OffboardControlMode>(
        "/fmu/offboard_control_mode/in", qos_history_depth)),
      trajectory_pub_(create_publisher<TrajectorySetpoint>(
        "/fmu/trajectory_setpoint/in", qos_history_depth)),
      vehicle_cmd_pub_(create_publisher<VehicleCommand>(
        "/fmu/vehicle_command/in", qos_history_depth)) {}

void ControllerBase::VehicleOdometryCallback(const VehicleOdometry::SharedPtr msg) const {
  // position
  world_t_body_ = {msg->x, msg->y, msg->z};

  // orientation
  world_R_body_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);

  // velocity
  v_body_ = {msg->vx, msg->vy, msg->vz};

  // anguler velocity
  w_body_ = {msg->rollspeed, msg->pitchspeed, msg->yawspeed};

  // re-publish pose for rviz visualization
  PoseStamped pose_msg{};
  pose_msg.header.frame_id = "world_ned";
  pose_msg.header.stamp = rclcpp::Clock().now();
  pose_msg.pose.position = tf2::toMsg(world_t_body_);
  pose_msg.pose.orientation = tf2::toMsg(world_R_body_);
  pose_pub_->publish(pose_msg);
}

void ControllerBase::TimeSyncCallback(const Timesync::SharedPtr msg) const {
  timestamp_synced_ = msg->timestamp;
}

void ControllerBase::SetPosition(const Eigen::Vector3d& position, const double& yaw) const {
  const uint64_t timestamp = rclcpp::Clock().now().nanoseconds() / 1000;

  OffboardControlMode mode_msg{};
  mode_msg.timestamp = timestamp;
  mode_msg.position = true;

  TrajectorySetpoint traj_msg{};
  traj_msg.timestamp = timestamp;
  traj_msg.position[0] = position.x();
  traj_msg.position[1] = position.y();
  traj_msg.position[2] = position.z();
  traj_msg.yaw = yaw;

  offboard_mode_pub_->publish(mode_msg);
  trajectory_pub_->publish(traj_msg);

  RCLCPP_DEBUG(this->get_logger(), "Sent trajectory setpoint");
}

inline VehicleCommand DefaultVehicleCommand() {
  VehicleCommand msg{};
  msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  return msg;
}

void ControllerBase::EnableOffboardCtrl() const {
  constexpr float base_mode_custom = 1;
  constexpr float main_mode_offboard = 6;

  VehicleCommand msg = DefaultVehicleCommand();
  msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  msg.param1 = base_mode_custom;
  msg.param2 = main_mode_offboard;

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Offboard control enabled");
}

void ControllerBase::Arm() const {
  VehicleCommand msg = DefaultVehicleCommand();
  msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg.param1 = VehicleCommand::ARMING_ACTION_ARM;

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Armed");
}

void ControllerBase::Disarm() const {
  VehicleCommand msg = DefaultVehicleCommand();
  msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg.param1 = VehicleCommand::ARMING_ACTION_DISARM;

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Disarmed");
}

} // namespace asl

