#pragma once

#include <atomic>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>


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
  // working in ROS default frame NWU -- North-West-Up

  // position
  mutable Eigen::Vector3d world_t_body_; // pose position

  // linear velocity
  mutable Eigen::Vector3d v_world_; // velocity in inertial frame
  mutable Eigen::Vector3d v_body_;  // velocity in inertial NED frame

  // orientation
  mutable Eigen::Quaterniond world_R_body_;

  // angular velocity
  mutable Eigen::Vector3d w_world_;
  mutable Eigen::Vector3d w_body_;

  // Synced Timestamp
  mutable std::atomic<uint64_t> timestamp_synced_;

  // PX4 message subscriptions
  const rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  const rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr sub_timesync_;

  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

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
};

} // namespace asl
