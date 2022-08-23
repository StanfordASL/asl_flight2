#include "asl_flight2/controller_base.h"

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
      setpoint_loop_timer_(nullptr),
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
        "/fmu/vehicle_command/in", qos_history_depth)) {

  ob_ctrl_mode_.position = false;
  ob_ctrl_mode_.velocity = false;
  ob_ctrl_mode_.acceleration = false;
  ob_ctrl_mode_.attitude = false;
  ob_ctrl_mode_.body_rate = false;
  ob_ctrl_mode_.actuator = false;
}

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

void ControllerBase::SetPosition(const Eigen::Vector3d& position, const double& yaw) {
  ob_setpoint_.position[0] = position.x();
  ob_setpoint_.position[1] = position.y();
  ob_setpoint_.position[2] = position.z();
  ob_setpoint_.yaw = yaw;
}

void ControllerBase::SetVelocity(const Eigen::Vector3d& velocity, const double& yaw_rate) {
  ob_setpoint_.velocity[0] = velocity.x();
  ob_setpoint_.velocity[1] = velocity.y();
  ob_setpoint_.velocity[2] = velocity.z();
  ob_setpoint_.yawspeed = yaw_rate;
}

void ControllerBase::SetAltitude(const double& altitude) {
  ob_setpoint_.position[2] = -altitude;
}

void ControllerBase::SetTrajCtrlMode(const trajectory_ctrl_mode_e& mode) {
  switch (mode) {
  // position control
  case POSITION:
    ob_setpoint_.velocity[0] = NAN;
    ob_setpoint_.velocity[1] = NAN;
    ob_setpoint_.velocity[2] = NAN;
    ob_setpoint_.yawspeed = NAN;
    __attribute__ ((fallthrough));
  case POSITION_VELOCITY:
    ob_setpoint_.acceleration[0] = NAN;
    ob_setpoint_.acceleration[1] = NAN;
    ob_setpoint_.acceleration[2] = NAN;
    __attribute__ ((fallthrough));
  case POSITION_VELOCITY_ACCELERATION:
    ob_ctrl_mode_.position = true;
    break;

  // velocity control
  case VELOCITY:
    ob_setpoint_.position[2] = NAN;
    __attribute__ ((fallthrough));
  case VELOCITY_ALTITUDE:
    ob_setpoint_.acceleration[0] = NAN;
    ob_setpoint_.acceleration[1] = NAN;
    ob_setpoint_.acceleration[2] = NAN;
    __attribute__ ((fallthrough));
  case VELOCITY_ALTITUDE_ACCELERATION:
    ob_setpoint_.position[0] = NAN;
    ob_setpoint_.position[1] = NAN;
    ob_setpoint_.yaw = NAN;
    ob_ctrl_mode_.position = false;
    ob_ctrl_mode_.velocity = true;
    break;

  // acceleration control
  case ACCELERATION:
    ob_ctrl_mode_.position = false;
    ob_ctrl_mode_.velocity = false;
    ob_ctrl_mode_.acceleration = true;

    ob_setpoint_.position[0] = NAN;
    ob_setpoint_.position[1] = NAN;
    ob_setpoint_.position[2] = NAN;
    ob_setpoint_.velocity[0] = NAN;
    ob_setpoint_.velocity[1] = NAN;
    ob_setpoint_.velocity[2] = NAN;
    break;
  }

  setpoint_loop_timer_ = this->create_wall_timer(
    50ms, std::bind(&ControllerBase::SetpointCallback, this));
}

void ControllerBase::StopSetpointLoop() {
  SetHoldMode();

  ob_ctrl_mode_.position = false;
  ob_ctrl_mode_.velocity = false;
  ob_ctrl_mode_.acceleration = false;
  ob_ctrl_mode_.attitude = false;
  ob_ctrl_mode_.body_rate = false;
  ob_ctrl_mode_.actuator = false;

  setpoint_loop_timer_ = this->create_wall_timer(
    1s, std::bind(&ControllerBase::DummyCallback, this));
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

// @see https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/px4_custom_mode.h
// for the custom modes used below
void ControllerBase::SetFlightMode(float main_mode_offboard, float sub_mode) const {
  constexpr float base_mode_custom = 1;

  VehicleCommand msg = DefaultVehicleCommand();
  msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  msg.param1 = base_mode_custom;
  msg.param2 = main_mode_offboard;
  msg.param3 = sub_mode;

  vehicle_cmd_pub_->publish(msg);
}

void ControllerBase::SetOffboardMode() const {
  constexpr float main_mode_offboard = 6;
  SetFlightMode(main_mode_offboard);

  RCLCPP_INFO(this->get_logger(), "Mode switch: offboard control");
}

void ControllerBase::SetHoldMode() const {
  constexpr float main_mode_auto = 4;
  constexpr float sub_mode_loiter = 3;
  SetFlightMode(main_mode_auto, sub_mode_loiter);

  RCLCPP_INFO(this->get_logger(), "Mode switch: hold");
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

void ControllerBase::Takeoff() {
  VehicleCommand msg = DefaultVehicleCommand();
  msg.command = VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  msg.param1 = NAN;
  msg.param2 = NAN;
  msg.param3 = NAN;
  msg.param4 = NAN;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = 1.0; // any non-NAN number is ok

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Takeoff initiated");
}

void ControllerBase::Land() {
  StopSetpointLoop(); // stop setpoint loop

  VehicleCommand msg = DefaultVehicleCommand();
  msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
  msg.param1 = NAN;
  msg.param2 = NAN;
  msg.param3 = NAN;
  msg.param4 = NAN;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = 0.0; // any non-NAN number is ok

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Land initiated");
}

void ControllerBase::SetpointCallback() {
  const uint64_t timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
  ob_ctrl_mode_.timestamp = timestamp;
  offboard_mode_pub_->publish(ob_ctrl_mode_);

  ob_setpoint_.timestamp = timestamp;
  trajectory_pub_->publish(ob_setpoint_);

  // lazily enable offboard control mode
  if (!vehicle_ctrl_mode_.flag_control_offboard_enabled) {
    SetOffboardMode();
  }
}

void ControllerBase::DummyCallback() {} // no op

} // namespace asl

