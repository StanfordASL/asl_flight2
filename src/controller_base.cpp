#include "asl_flight2/controller_base.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

ControllerBase::ControllerBase(const std::string& node_name, const size_t qos_history_depth_)
    : rclcpp::Node(node_name),
      sub_odom_(create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/vehicle_odometry/out",
        qos_history_depth_,
        std::bind(&ControllerBase::VehicleOdometryCallback, this, _1)
      )),
      sub_timesync_(create_subscription<px4_msgs::msg::Timesync>(
        "/fmu/timesync/out",
        qos_history_depth_,
        std::bind(&ControllerBase::TimeSyncCallback, this, _1)
      )) {
    // Create PX4 message subscriptions
    // TODO: understand ROS2 QoS profiles better
}

void ControllerBase::callbackPositionAndVelocity(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    pos_ned_ = {msg->x, msg->y, msg->z};
    vel_ned_ = {msg->vx, msg->vy, msg->vz};
    pos_nwu_ = transform_flip_z(pos_ned_);
    vel_nwu_ = transform_flip_z(vel_ned_);
    vel_frd_ = att_ned_frd_.conjugate() * vel_ned_;
    vel_flu_ = transform_flip_z(vel_frd_);
}

void ControllerBase::callbackAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    att_ned_frd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
    att_ned_frd_.normalize();
    att_nwu_flu_ = ROT_NWU_NED * att_ned_frd_ * ROT_FRD_FLU;
}

void ControllerBase::callbackAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
    angvel_frd_ = {msg->xyz[0], msg->xyz[1], msg->xyz[2]};
    angvel_flu_ = transform_flip_z(angvel_frd_);
}

void ControllerBase::TimeSyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) const {
    timestamp_.store(msg->timestamp);
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerBase>("controller_base"));
    rclcpp::shutdown();
    return 0;
}
