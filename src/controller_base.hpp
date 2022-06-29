#pragma once

#include <atomic>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <px4_ros_com/frame_transforms.h>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
// #include <px4_msgs/msg/actuator_direct_control.hpp>


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
    Eigen::Vector3d pos_ned_;   // position in inertial NED frame
    Eigen::Vector3d pos_nwu_;   // position in inertial NWU frame

    // Linear velocity
    Eigen::Vector3d vel_ned_;   // velocity in inertial NED frame
    Eigen::Vector3d vel_nwu_;   // velocity in inertial NWU frame
    Eigen::Vector3d vel_frd_;   // velocity in body FRD frame
    Eigen::Vector3d vel_flu_;   // velocity in body FLU frame

    // Attitude (NOTE: Eigen quaternions are stored as [x, y, z, w])
    Eigen::Quaterniond att_ned_frd_;
    Eigen::Quaterniond att_nwu_flu_;

    // Angular velocity (FLU w.r.t. NWU, and FRD w.r.t. NED)
    Eigen::Vector3d angvel_flu_;
    Eigen::Vector3d angvel_frd_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // PX4 message subscriptions
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_pos_vel_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_att_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr sub_angvel_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr sub_timesync_;

    // Synced Timestamp
    std::atomic<uint64_t> timestamp_;

    // Callbacks

    /**
     * @brief      { function_description }
     */
    virtual void callbackTimer();

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
    virtual void callbackTimeSync(const px4_msgs::msg::Timesync::UniquePtr msg);
};
