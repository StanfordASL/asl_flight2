#pragma once

#include <atomic>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <px4_ros_com/frame_transforms.h>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>


const Eigen::Quaterniond ROT_FRD_FLU
    = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(-M_PI, 0.0, 0.0);

const Eigen::Quaterniond ROT_NWU_NED
    = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);

/**
 * @brief      { function_description }
 *
 * @param[in]  v     { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
inline Eigen::Vector3d transform_flip_z(const Eigen::Vector3d& v) {
    return {v(0), -v(1), -v(2)};
}

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
	explicit ControllerBase(const std::string& node_name, const size_t qos_history_depth_ = 10);

    /**
     * @brief      Destroys the object.
     */
	virtual ~ControllerBase() = default;

protected:
    double loop_freq_;          // control loop frequency (Hz)

    // Frame acronyms:
    //  NED -- North-East-Down
    //  NWD -- North-West-Down
    //  FRD -- Forward-Right-Down
    //  FLU -- Forward-Left-Up

    // Position
    mutable Eigen::Vector3d pos_ned_;   // position in inertial NED frame
    mutable Eigen::Vector3d pos_nwu_;   // position in inertial NWU frame

    // Linear velocity
    mutable Eigen::Vector3d vel_ned_;   // velocity in inertial NED frame
    mutable Eigen::Vector3d vel_nwu_;   // velocity in inertial NWU frame
    mutable Eigen::Vector3d vel_frd_;   // velocity in body FRD frame
    mutable Eigen::Vector3d vel_flu_;   // velocity in body FLU frame

    // Attitude (NOTE: Eigen quaternions are stored as [x, y, z, w])
    mutable Eigen::Quaterniond att_ned_frd_;
    mutable Eigen::Quaterniond att_nwu_flu_;

    // Angular velocity (FLU w.r.t. NWU, and FRD w.r.t. NED)
    mutable Eigen::Vector3d angvel_flu_;
    mutable Eigen::Vector3d angvel_frd_;

    // Synced Timestamp
    mutable std::atomic<uint64_t> timestamp_;

    // PX4 message subscriptions
    const rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    const rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr sub_timesync_;

    // Callbacks

    /**
     * @brief callback handler for reading state from the flight controller
     *
     * @param msg message containing odometry data
     */
    virtual void VehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    /**
     * @brief      { function_description }
     *
     * @param[in]  msg   The message
     */
    virtual void callbackPositionAndVelocity(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    /**
     * @brief      { function_description }
     *
     * @param[in]  msg   The message
     */
    virtual void callbackAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);

    /**
     * @brief      { function_description }
     *
     * @param[in]  msg   The message
     */
    virtual void callbackAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);

    /**
     * @brief      { function_description }
     *
     * @param[in]  msg   The message
     */
    virtual void TimeSyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) const;
};
