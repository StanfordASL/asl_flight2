#pragma once

#include <atomic>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>


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
  mutable std::atomic<uint64_t> timestamp_synced_;

  // PX4 message subscriptions
  const rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  const rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr sub_timesync_;

  // PX4 message publishers
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  const rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  const rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

  // Callbacks

  /**
   * @brief callback handler for reading state from the flight controller
   *
   * @param msg message containing odometry data
   */
  void VehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const;

  /**
  * @brief      { function_description }
  *
  * @param[in]  msg   The message
  */
  void TimeSyncCallback(const px4_msgs::msg::Timesync::SharedPtr msg) const;

  void SetPosition(const Eigen::Vector3d& position, const double& yaw = 0) const;

  void EnableOffboardCtrl() const;

  void Arm() const;

  void Disarm() const;
};

} // namespace asl
