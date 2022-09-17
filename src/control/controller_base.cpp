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

#include "asl_flight2/controller_base.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros_com/frame_transforms.h>

#include <tf2_eigen/tf2_eigen.h>

#include <chrono>
#include <functional>
#include <string>

namespace asl
{

using namespace std::chrono_literals;

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleControlMode;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleRatesSetpoint;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using std::placeholders::_1;

ControllerBase::ControllerBase(const std::string & node_name, const size_t qos_history_depth)
: rclcpp::Node(node_name),
  setpoint_loop_timer_(nullptr),
  ctrl_mode_loop_timer_(nullptr),
  sub_ctrl_mode_(create_subscription<VehicleControlMode>(
      "/fmu/vehicle_control_mode/out",
      qos_history_depth,
      [this](const VehicleControlMode::SharedPtr msg) {vehicle_ctrl_mode_ = *msg;}
    )),
  sub_odom_(
    create_subscription<VehicleOdometry>(
      "/fmu/vehicle_odometry/out",
      qos_history_depth,
      std::bind(&ControllerBase::VehicleOdometryCallback, this, _1)
    )),
  sub_status_(
    create_subscription<VehicleStatus>(
      "/fmu/vehicle_status/out",
      qos_history_depth,
      [this](const VehicleStatus::SharedPtr msg) {vehicle_status_ = *msg;}
    )),
  pose_pub_(create_publisher<PoseWithCovarianceStamped>("pose", qos_history_depth)),
  offboard_mode_pub_(
    create_publisher<OffboardControlMode>(
      "/fmu/offboard_control_mode/in", qos_history_depth)),
  trajectory_pub_(
    create_publisher<TrajectorySetpoint>(
      "/fmu/trajectory_setpoint/in", qos_history_depth)),
  attitude_pub_(
    create_publisher<VehicleAttitudeSetpoint>(
      "/fmu/vehicle_attitude_setpoint/in", qos_history_depth)),
  rates_pub_(
    create_publisher<VehicleRatesSetpoint>(
      "/fmu/vehicle_rates_setpoint/in", qos_history_depth)),
  vehicle_cmd_pub_(
    create_publisher<VehicleCommand>(
      "/fmu/vehicle_command/in", qos_history_depth)) {
  ob_ctrl_mode_.position = false;
  ob_ctrl_mode_.velocity = false;
  ob_ctrl_mode_.acceleration = false;
  ob_ctrl_mode_.attitude = false;
  ob_ctrl_mode_.body_rate = false;
  ob_ctrl_mode_.actuator = false;
}

void ControllerBase::VehicleOdometryCallback(const VehicleOdometry::SharedPtr msg)
{
  vehicle_state_.timetsamp = rclcpp::Time(msg->timestamp_sample * 1e3);

  // position
  vehicle_state_.world_T_body.t = {msg->position[0], msg->position[1], msg->position[2]};

  // orientation
  vehicle_state_.world_T_body.R =
    px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);

  // velocity
  vehicle_state_.twist_body.v = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};

  // anguler velocity
  vehicle_state_.twist_body.w =
  {msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[1]};

  // re-publish pose for rviz visualization
  PoseWithCovarianceStamped viz_msg{};
  viz_msg.header.frame_id = "world_ned";
  viz_msg.header.stamp = vehicle_state_.timetsamp;

  viz_msg.pose.pose.position = tf2::toMsg(vehicle_state_.world_T_body.t);
  viz_msg.pose.pose.orientation = tf2::toMsg(vehicle_state_.world_T_body.R);
  for (int i = 0; i < 3; ++i) {
    viz_msg.pose.covariance[i * 6 + i] = msg->position_variance[i];
    viz_msg.pose.covariance[(i + 3) * 6 + i + 3] = msg->orientation_variance[i];
  }
  pose_pub_->publish(viz_msg);
}

void ControllerBase::SetPosition(const Eigen::Vector3d & position, const double & yaw)
{
  ob_traj_setpoint_.position[0] = position.x();
  ob_traj_setpoint_.position[1] = position.y();
  ob_traj_setpoint_.position[2] = position.z();
  ob_traj_setpoint_.yaw = yaw;
}

void ControllerBase::SetVelocity(const Eigen::Vector3d & velocity, const double & yaw_rate)
{
  ob_traj_setpoint_.velocity[0] = velocity.x();
  ob_traj_setpoint_.velocity[1] = velocity.y();
  ob_traj_setpoint_.velocity[2] = velocity.z();
  ob_traj_setpoint_.yawspeed = yaw_rate;
}

void ControllerBase::SetAltitude(const double & altitude)
{
  ob_traj_setpoint_.position[2] = -altitude;
}

void ControllerBase::SetAttitude(
  const Eigen::Quaterniond & attitude,
  const double & thrust,
  const double & yaw_rate)
{
  double safe_thrust = thrust;
  if (thrust < 0 || thrust > 1) {
    safe_thrust = 0.0;
    RCLCPP_ERROR(
      this->get_logger(),
      "provided thrust %d is not normalized to [0, 1], using 0 thrust", thrust);
  }

  ob_attitude_setpoint_.q_d[0] = attitude.w();
  ob_attitude_setpoint_.q_d[1] = attitude.x();
  ob_attitude_setpoint_.q_d[2] = attitude.y();
  ob_attitude_setpoint_.q_d[3] = attitude.z();
  ob_attitude_setpoint_.thrust_body[2] = -safe_thrust;
  ob_attitude_setpoint_.yaw_sp_move_rate = yaw_rate;
}

void ControllerBase::SetBodyRate(const Eigen::Vector3d & rates, const double & thrust)
{
  double safe_thrust = thrust;
  if (thrust < 0 || thrust > 1) {
    safe_thrust = 0.0;
    RCLCPP_ERROR(
      this->get_logger(),
      "provided thrust %d is not normalized to [0, 1], using 0 thrust", thrust);
  }

  ob_rate_setpoint_.roll = rates.x();
  ob_rate_setpoint_.pitch = rates.y();
  ob_rate_setpoint_.yaw = rates.z();
  ob_rate_setpoint_.thrust_body[2] = -safe_thrust;
}

void ControllerBase::SetTrajCtrlMode(const TrajectoryControlMode & mode)
{
  if (ob_ctrl_mode_.position) {
    RCLCPP_WARN(this->get_logger(), "SetTrajCtrlMode ignore: already eanbled");
    return;
  }

  switch (mode) {
    // position control
    case POSITION:
      ob_traj_setpoint_.velocity[0] = NAN;
      ob_traj_setpoint_.velocity[1] = NAN;
      ob_traj_setpoint_.velocity[2] = NAN;
      ob_traj_setpoint_.yawspeed = NAN;
      __attribute__ ((fallthrough));
    case POSITION_VELOCITY:
      ob_traj_setpoint_.acceleration[0] = NAN;
      ob_traj_setpoint_.acceleration[1] = NAN;
      ob_traj_setpoint_.acceleration[2] = NAN;
      __attribute__ ((fallthrough));
    case POSITION_VELOCITY_ACCELERATION:
      ob_ctrl_mode_.position = true;
      break;

    // velocity control
    case VELOCITY:
      ob_traj_setpoint_.position[2] = NAN;
      __attribute__ ((fallthrough));
    case VELOCITY_ALTITUDE:
      ob_traj_setpoint_.acceleration[0] = NAN;
      ob_traj_setpoint_.acceleration[1] = NAN;
      ob_traj_setpoint_.acceleration[2] = NAN;
      __attribute__ ((fallthrough));
    case VELOCITY_ALTITUDE_ACCELERATION:
      ob_traj_setpoint_.position[0] = NAN;
      ob_traj_setpoint_.position[1] = NAN;
      ob_traj_setpoint_.yaw = NAN;
      ob_ctrl_mode_.position = false;
      ob_ctrl_mode_.velocity = true;
      break;

    // acceleration control
    case ACCELERATION:
      ob_ctrl_mode_.position = false;
      ob_ctrl_mode_.velocity = false;
      ob_ctrl_mode_.acceleration = true;

      ob_traj_setpoint_.position[0] = NAN;
      ob_traj_setpoint_.position[1] = NAN;
      ob_traj_setpoint_.position[2] = NAN;
      ob_traj_setpoint_.velocity[0] = NAN;
      ob_traj_setpoint_.velocity[1] = NAN;
      ob_traj_setpoint_.velocity[2] = NAN;
      break;
  }

  setpoint_loop_timer_ = this->create_wall_timer(
    50ms, std::bind(&ControllerBase::TrajSetpointCallback, this));
  ctrl_mode_loop_timer_ = this->create_wall_timer(
    100ms, std::bind(&ControllerBase::OffboardControlModeCallback, this));

  RCLCPP_INFO(this->get_logger(), "Trajectory control mode enabled");
}

void ControllerBase::SetAttitudeCtrlMode()
{
  if (!ob_ctrl_mode_.position &&
    !ob_ctrl_mode_.velocity &&
    !ob_ctrl_mode_.acceleration &&
    ob_ctrl_mode_.attitude)
  {
    RCLCPP_WARN(this->get_logger(), "SetAttitudeCtrlMode ignore: already eanbled");
    return;
  }

  ob_ctrl_mode_.position = false;
  ob_ctrl_mode_.velocity = false;
  ob_ctrl_mode_.acceleration = false;
  ob_ctrl_mode_.attitude = true;

  ob_attitude_setpoint_.thrust_body[0] = 0.0f;
  ob_attitude_setpoint_.thrust_body[1] = 0.0f;

  setpoint_loop_timer_ = this->create_wall_timer(
    20ms, std::bind(&ControllerBase::AttitudeSetpointCallback, this));
  ctrl_mode_loop_timer_ = this->create_wall_timer(
    100ms, std::bind(&ControllerBase::OffboardControlModeCallback, this));

  RCLCPP_INFO(this->get_logger(), "Attitude control mode enabled");
}

void ControllerBase::SetBodyRateCtrlMode()
{
  if (!ob_ctrl_mode_.position &&
    !ob_ctrl_mode_.velocity &&
    !ob_ctrl_mode_.acceleration &&
    !ob_ctrl_mode_.attitude &&
    ob_ctrl_mode_.body_rate)
  {
    RCLCPP_WARN(this->get_logger(), "SetBodyRateCtrlMode ignore: already eanbled");
    return;
  }

  ob_ctrl_mode_.position = false;
  ob_ctrl_mode_.velocity = false;
  ob_ctrl_mode_.acceleration = false;
  ob_ctrl_mode_.attitude = false;
  ob_ctrl_mode_.body_rate = true;

  ob_rate_setpoint_.thrust_body[0] = 0.0f;
  ob_rate_setpoint_.thrust_body[1] = 0.0f;

  setpoint_loop_timer_ = this->create_wall_timer(
    10ms, std::bind(&ControllerBase::RatesSetpointCallback, this));
  ctrl_mode_loop_timer_ = this->create_wall_timer(
    100ms, std::bind(&ControllerBase::OffboardControlModeCallback, this));

  RCLCPP_INFO(this->get_logger(), "Body rate control mode enabled");
}

void ControllerBase::StopSetpointLoop()
{
  if (setpoint_loop_timer_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "No active setpoint loop");
    return;
  }

  SetHoldMode();

  ob_ctrl_mode_.position = false;
  ob_ctrl_mode_.velocity = false;
  ob_ctrl_mode_.acceleration = false;
  ob_ctrl_mode_.attitude = false;
  ob_ctrl_mode_.body_rate = false;
  ob_ctrl_mode_.actuator = false;

  setpoint_loop_timer_ = nullptr;
  ctrl_mode_loop_timer_ = this->create_wall_timer(
    1s, std::bind(&ControllerBase::DummyCallback, this));
}

void ControllerBase::SetDefaultVehicleCommand(VehicleCommand * msg) const
{
  msg->timestamp = this->now().nanoseconds() / 1000;
  msg->target_system = 1;
  msg->target_component = 1;
  msg->source_system = 1;
  msg->source_component = 1;
  msg->from_external = true;
}

// @see https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/px4_custom_mode.h
// for the custom modes used below
void ControllerBase::SetFlightMode(float main_mode_offboard, float sub_mode)
{
  constexpr float base_mode_custom = 1;

  VehicleCommand msg{};
  this->SetDefaultVehicleCommand(&msg);
  msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  msg.param1 = base_mode_custom;
  msg.param2 = main_mode_offboard;
  msg.param3 = sub_mode;

  vehicle_cmd_pub_->publish(msg);
}

void ControllerBase::SetOffboardMode()
{
  constexpr float main_mode_offboard = 6;
  SetFlightMode(main_mode_offboard);

  RCLCPP_INFO(this->get_logger(), "Mode switch: offboard control");
}

void ControllerBase::SetHoldMode()
{
  constexpr float main_mode_auto = 4;
  constexpr float sub_mode_loiter = 3;
  SetFlightMode(main_mode_auto, sub_mode_loiter);

  RCLCPP_INFO(this->get_logger(), "Mode switch: hold");
}

void ControllerBase::Arm()
{
  if (Armed()) {
    RCLCPP_WARN(this->get_logger(), "Arm ignored: vehicle already armed");
    return;
  }

  VehicleCommand msg{};
  this->SetDefaultVehicleCommand(&msg);
  msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg.param1 = VehicleCommand::ARMING_ACTION_ARM;

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Armed");
}

void ControllerBase::Disarm()
{
  if (IsAirborne()) {
    RCLCPP_WARN(this->get_logger(), "Disarm ignored: vehicle in air");
    return;
  } else if (!Armed()) {
    RCLCPP_WARN(this->get_logger(), "Disarm ignored: vehicle not armed");
    return;
  }

  VehicleCommand msg{};
  this->SetDefaultVehicleCommand(&msg);
  msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg.param1 = VehicleCommand::ARMING_ACTION_DISARM;

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Disarmed");
}

void ControllerBase::Takeoff()
{
  if (!Armed()) {
    RCLCPP_WARN(this->get_logger(), "Takeoff ignored: vehicle not armed");
    return;
  } else if (IsAirborne()) {
    RCLCPP_WARN(this->get_logger(), "Takeoff ignored: vehicle already in air");
    return;
  } else if (TakeoffInProgress()) {
    RCLCPP_WARN(this->get_logger(), "Takeoff ignored: takeoff in progress");
    return;
  }

  VehicleCommand msg{};
  this->SetDefaultVehicleCommand(&msg);
  msg.command = VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  msg.param1 = NAN;
  msg.param2 = NAN;
  msg.param3 = NAN;
  msg.param4 = NAN;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = 1.0;  // any non-NAN number is ok

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Takeoff initiated");
}

void ControllerBase::Land()
{
  if (!Armed()) {
    RCLCPP_WARN(this->get_logger(), "Land ignored: vehicle not armed");
    return;
  } else if (!IsAirborne()) {
    RCLCPP_WARN(this->get_logger(), "Land ignored: vehicle already on the ground");
    return;
  } else if (LandingInProgress()) {
    RCLCPP_WARN(this->get_logger(), "Land ignored: landing in progress");
    return;
  }

  StopSetpointLoop();  // stop setpoint loop

  VehicleCommand msg{};
  this->SetDefaultVehicleCommand(&msg);
  msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
  msg.param1 = NAN;
  msg.param2 = NAN;
  msg.param3 = NAN;
  msg.param4 = NAN;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = 0.0;  // any non-NAN number is ok

  vehicle_cmd_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Land initiated");
}

bool ControllerBase::Armed() const
{
  return vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
}

bool ControllerBase::TakeoffInProgress() const
{
  return vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF;
}

bool ControllerBase::LandingInProgress() const
{
  return vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
}

bool ControllerBase::IsAirborne() const
{
  return vehicle_status_.takeoff_time != 0;
}

void ControllerBase::TrajSetpointCallback()
{
  ob_traj_setpoint_.timestamp = this->now().nanoseconds() / 1000;
  trajectory_pub_->publish(ob_traj_setpoint_);
}

void ControllerBase::AttitudeSetpointCallback()
{
  ob_attitude_setpoint_.timestamp = this->now().nanoseconds() / 1000;
  attitude_pub_->publish(ob_attitude_setpoint_);
}

void ControllerBase::RatesSetpointCallback()
{
  ob_rate_setpoint_.timestamp = this->now().nanoseconds() / 1000;
  rates_pub_->publish(ob_rate_setpoint_);
}

void ControllerBase::OffboardControlModeCallback()
{
  ob_ctrl_mode_.timestamp = this->now().nanoseconds() / 1000;
  offboard_mode_pub_->publish(ob_ctrl_mode_);

  // lazily enable offboard control mode
  if (!vehicle_ctrl_mode_.flag_control_offboard_enabled) {
    SetOffboardMode();
  }
}

void ControllerBase::DummyCallback() {}  // no op

}  // namespace asl
