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

#ifndef ASL_FLIGHT2__CONTROLLER_BASE_HPP_
#define ASL_FLIGHT2__CONTROLLER_BASE_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/timesync.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

namespace asl
{

/**
 * @brief drone state data structure working in PX4 default frame NED -- North-East-Down
 */
struct VehicleState
{
  /**
   * @brief default constructor
   */
  VehicleState();

  /**
   * @brief construct from VehicleOdometry message
   *
   * @param odom VehicleOdometry message
   */
  explicit VehicleState(const px4_msgs::msg::VehicleOdometry & odom);

  // timestamp
  rclcpp::Time timestamp;

  // pose
  Eigen::Vector3d world_t_body;     // translation
  Eigen::Quaterniond world_R_body;  // rotation

  // twist
  Eigen::Vector3d v_body;           // linear velocity
  Eigen::Vector3d w_body;           // angular velocity
};

/**
 * @brief      This class describes a controller base.
 */
class ControllerBase : public rclcpp::Node
{
public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  node_name  The node name
   */
  explicit ControllerBase(const std::string & node_name, const size_t qos_history_depth = 10);

  /**
   * @brief      Destroys the object.
   */
  virtual ~ControllerBase() = default;

  /**
   * @brief get the latest vehicle state
   *
   * @return latest VehicleState
   */
  inline VehicleState GetVehicleState() const
  {
    std::lock_guard<std::mutex> lock(vehicle_odom_mtx_);
    if (vehicle_odom_) {
      return VehicleState(*vehicle_odom_);
    } else {
      return VehicleState();
    }
  }

protected:
  // vehicle states
  px4_msgs::msg::VehicleOdometry::SharedPtr vehicle_odom_;
  px4_msgs::msg::VehicleControlMode::SharedPtr vehicle_ctrl_mode_;
  px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;
  mutable std::mutex vehicle_odom_mtx_;
  mutable std::mutex vehicle_ctrl_mode_mtx_;
  mutable std::mutex vehicle_status_mtx_;

  // control mode sent together with setpoint messages
  px4_msgs::msg::OffboardControlMode ob_ctrl_mode_;

  // control setpoint messages
  px4_msgs::msg::TrajectorySetpoint ob_traj_setpoint_;
  px4_msgs::msg::VehicleAttitudeSetpoint ob_attitude_setpoint_;
  px4_msgs::msg::VehicleRatesSetpoint ob_rate_setpoint_;

  // timer for periodic sending setpoint message
  rclcpp::TimerBase::SharedPtr setpoint_loop_timer_;
  rclcpp::TimerBase::SharedPtr ctrl_mode_loop_timer_;

  // callback groups
  const rclcpp::CallbackGroup::SharedPtr parallel_cb_group_;
  const rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  const rclcpp::SubscriptionOptions parallel_sub_options_;

  // PX4 message subscriptions
  const rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr sub_ctrl_mode_;
  const rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  const rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;

  // PX4 message publishers
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  const rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  const rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  const rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
  const rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_pub_;
  const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

  // control modes
  enum TrajectoryControlMode
  {
    POSITION,
    POSITION_VELOCITY,
    POSITION_VELOCITY_ACCELERATION,
    VELOCITY,
    VELOCITY_ALTITUDE,
    VELOCITY_ALTITUDE_ACCELERATION,
    ACCELERATION,
  };

  /**
   * @brief send target position to flight controller
   *
   * @param position  3D position in global NED frame
   * @param yaw       yaw angle in radians (NED -> clockwise is positive)
   */
  void SetPosition(const Eigen::Vector3d & position, const double & yaw = 0);

  /**
   * @brief send target velocity to flight controller
   *
   * @param velocity  3D velocity in global NED frame
   * @param yaw_rate  yaw velocity in rad/s (NED -> clockwise is positive)
   */
  void SetVelocity(const Eigen::Vector3d & velocity, const double & yaw_rate = 0);

  /**
   * @brief send target altitude to flight controller
   *
   * @param altitude altitude in [m] (positive going upwards)
   */
  void SetAltitude(const double & altitude);

  /**
   * @brief setn target attitude to flight controller
   *
   * @param attitude  target orientation
   * @param thrust    target thrust normalized to [0, 1]
   * @param yaw_rate  yaw velocity in rad/s (NED -> clockwise is positive)
   */
  void SetAttitude(
    const Eigen::Quaterniond & attitude, const double & thrust,
    const double & yaw_rate = NAN);

  /**
   * @brief set body rate control
   *
   * @param rates   body frame angular rate in NED frame
   * @param thrust  thrust normalized to [0, 1]
   */
  void SetBodyRate(const Eigen::Vector3d & rates, const double & thrust);

  /**
   * @brief set trajectory setpoint control mode
   *
   * @param mode see TrajectoryControlMode
   */
  void SetTrajCtrlMode(const TrajectoryControlMode & mode);

  /**
   * @brief set attitude control mode
   */
  void SetAttitudeCtrlMode();

  /**
   * @brief set body rate control mode
   */
  void SetBodyRateCtrlMode();

  /**
   * @brief stop setpoint loop timer
   */
  void StopSetpointLoop();

  /**
   * @brief fill out basic structure of a vehicle_command message
   *
   * @param msg pointer to a VehicleCommand message to fill
   */
  void SetDefaultVehicleCommand(px4_msgs::msg::VehicleCommand * msg) const;

  /**
   * @brief set flight mode
   *
   * @param main_mode
   * @param sub_mode
   *
   * @see px4_custom_mode.h for mode definitions
   */
  void SetFlightMode(float main_mode, float sub_mode = NAN);

  /**
   * @brief switch to offboard control mode
   */
  void SetOffboardMode();

  /**
   * @brief switch to hold mode
   */
  void SetHoldMode();

  /**
   * @brief arm the drone
   */
  void Arm();

  /**
   * @brief disarm the drone
   */
  void Disarm();

  /**
   * @brief takeoff to fixed height
   * @see MIS_TAKEOFF_ALT in PX4 params
   */
  void Takeoff();

  /**
   * @brief land in place
   */
  void Land();

  /**
   * @brief check vehicle arm status
   *
   * @return true if armed, false otherwise
   */
  bool Armed() const;

  /**
   * @brief check if vehicle is in the process of taking off
   *
   * @return whether or not takeoff is in progress
   */
  bool TakeoffInProgress() const;

  /**
   * @brief check if vehicle is in the process of landing
   *
   * @return whether or not landing is in progress
   */
  bool LandingInProgress() const;

  /**
   * @brief check if vehicle is flying
   *
   * @return true if vehicle is flying, false otherwise
   */
  bool IsAirborne() const;

  /**
   * @brief check if offboard control is enabled
   *
   * @return true if offboard control is enabled
   */
  bool OffboardEnabled() const;

private:
  // Callbacks

  void VehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  void TrajSetpointCallback();

  void AttitudeSetpointCallback();

  void RatesSetpointCallback();

  void OffboardControlModeCallback();

  void DummyCallback();
};

}  // namespace asl

#endif  // ASL_FLIGHT2__CONTROLLER_BASE_HPP_
