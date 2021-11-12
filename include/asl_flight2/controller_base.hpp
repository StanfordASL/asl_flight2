#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <px4_ros_com/frame_transforms.h>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>


const Eigen::Quaterniond ROT_FRD_FLU = 
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(-M_PI, 0.0, 0.0);
const Eigen::Quaterniond ROT_NWU_NED = 
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);

void transform_flip_z(const Eigen::Vector3d& vec_in, Eigen::Vector3d& vec_out);

class ControllerBase : public rclcpp::Node {
	public:
		explicit ControllerBase(const std::string& node_name);
    	virtual ~ControllerBase();

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
        
        // Rotation (NOTE: Eigen quaternions are stored as [x, y, z, w])
        Eigen::Quaterniond rot_ned_frd_;
        Eigen::Quaterniond rot_nwu_flu_;

        // Angular velocity (FLU w.r.t. NWU, and FRD w.r.t. NED)
        Eigen::Vector3d angvel_frd_;
        Eigen::Vector3d angvel_flu_;

        // Timers
        rclcpp::TimerBase::SharedPtr timer_;

        // Callbacks
        virtual void callbackTimer();
        virtual void callbackPositionAndVelocity(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
        virtual void callbackAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
        virtual void callbackAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
        virtual void callbackTimeSync(const px4_msgs::msg::Timesync::UniquePtr msg);
};