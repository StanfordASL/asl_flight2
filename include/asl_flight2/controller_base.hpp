#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>


namespace asl {

/**
 * @brief      This class describes a controller base.
 */
class ControllerBase : public rclcpp::Node {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  node_name  The node name
   */
  explicit ControllerBase(const std::string& node_name, const size_t qos_history_depth = 10);

  /**
   * @brief      Destroys the object.
   */
  virtual ~ControllerBase() = default;

 protected:
  // working in PX4 default frame NED -- North-East-Down

  // position
  mutable Eigen::Vector3d world_t_body_;

  // orientation
  mutable Eigen::Quaterniond world_R_body_;

  // linear velocity
  mutable Eigen::Vector3d v_body_;

  // angular velocity
  mutable Eigen::Vector3d w_body_;

  // Synced Timestamp
  mutable uint64_t timestamp_synced_;

  // vehicle states
  mutable px4_msgs::msg::VehicleControlMode vehicle_ctrl_mode_;
  mutable px4_msgs::msg::VehicleStatus vehicle_status_;

  // control mode sent together with setpoint messages
  px4_msgs::msg::OffboardControlMode ob_ctrl_mode_;

  // control setpoint messages
  px4_msgs::msg::TrajectorySetpoint ob_traj_setpoint_;
  px4_msgs::msg::VehicleAttitudeSetpoint ob_attitude_setpoint_;
  px4_msgs::msg::VehicleRatesSetpoint ob_rate_setpoint_;

  // timer for periodic sending setpoint message
  rclcpp::TimerBase::SharedPtr setpoint_loop_timer_;
  rclcpp::TimerBase::SharedPtr ctrl_mode_loop_timer_;

  // PX4 message subscriptions
  const rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr sub_timesync_;
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
  enum trajectory_ctrl_mode_e {
    POSITION,
    POSITION_VELOCITY,
    POSITION_VELOCITY_ACCELERATION,
    VELOCITY,
    VELOCITY_ALTITUDE,
    VELOCITY_ALTITUDE_ACCELERATION,
    ACCELERATION,
  };

  // Callbacks

  /**
   * @brief callback handler for reading state from the flight controller
   *
   * @param msg message containing odometry data
   */
  void VehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const;

  /**
  * @brief callback for getting synchronized (10Hz) timestamp
  * @TODO(alvin): this does not seem necessary
  *
  * @param[in]  msg   The message
  */
  void TimeSyncCallback(const px4_msgs::msg::Timesync::SharedPtr msg) const;

  /**
   * @brief send target position to flight controller
   *
   * @param position  3D position in global NED frame
   * @param yaw       yaw angle in radians (NED -> clockwise is positive)
   */
  void SetPosition(const Eigen::Vector3d& position, const double& yaw = 0);

  /**
   * @brief send target velocity to flight controller
   *
   * @param velocity  3D velocity in global NED frame
   * @param yaw_rate  yaw velocity in rad/s (NED -> clockwise is positive)
   */
  void SetVelocity(const Eigen::Vector3d& velocity, const double& yaw_rate = 0);

  /**
   * @brief send target altitude to flight controller
   *
   * @param altitude altitude in [m] (positive going upwards)
   */
  void SetAltitude(const double& altitude);

  /**
   * @brief setn target attitude to flight controller
   *
   * @param attitude  target orientation
   * @param thrust    target thrust normalized to [0, 1]
   * @param yaw_rate  yaw velocity in rad/s (NED -> clockwise is positive)
   */
  void SetAttitude(const Eigen::Quaterniond& attitude, const double& thrust,
                   const double& yaw_rate = NAN);

  /**
   * @brief set body rate control
   *
   * @param rates   body frame angular rate in NED frame
   * @param thrust  thrust normalized to [0, 1]
   */
  void SetBodyRate(const Eigen::Vector3d& rates, const double& thrust);

  /**
   * @brief set trajectory setpoint control mode
   *
   * @param mode see trajectory_ctrl_mode_e
   */
  void SetTrajCtrlMode(const trajectory_ctrl_mode_e& mode);

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
   * @brief set flight mode
   *
   * @param main_mode
   * @param sub_mode
   *
   * @see px4_custom_mode.h for mode definitions
   */
  void SetFlightMode(float main_mode, float sub_mode = NAN) const;

  /**
   * @brief switch to offboard control mode
   */
  void SetOffboardMode() const;

  /**
   * @brief switch to hold mode
   */
  void SetHoldMode() const;

  /**
   * @brief arm the drone
   */
  void Arm() const;

  /**
   * @brief disarm the drone
   */
  void Disarm() const;

  /**
   * @brief takeoff to fixed height
   * @see MIS_TAKEOFF_ALT in PX4 params
   */
  void Takeoff();

  /**
   * @brief land in place
   */
  void Land();

 private:
  void TrajSetpointCallback();

  void AttitudeSetpointCallback();

  void RatesSetpointCallback();

  void OffboardControlModeCallback();

  void DummyCallback();
};

} // namespace asl
