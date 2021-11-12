#include "asl_flight2/controller_base.hpp"

void transform_flip_z(const Eigen::Vector3d& vec_in, Eigen::Vector3d& vec_out) {
    vec_out(0) = vec_in(0);
    vec_out(1) = -vec_in(1);
    vec_out(2) = -vec_in(2);
}

ControllerBase::ControllerBase(const std::string& node_name) : 
    rclcpp::Node(node_name) 
{
    // TODO.
}

ControllerBase::~ControllerBase() {};

void ControllerBase::callbackTimer() {
    RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
}

void ControllerBase::callbackPositionAndVelocity(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    pos_ned_(0) = msg->x;
    pos_ned_(1) = msg->y;
    pos_ned_(2) = msg->z;
    transform_flip_z(pos_ned_, pos_nwu_);

    vel_ned_(0) = msg->vx;
    vel_ned_(1) = msg->vy;
    vel_ned_(2) = msg->vz;
    transform_flip_z(vel_ned_, vel_nwu_);

    vel_frd_ = rot_ned_frd_.conjugate() * vel_ned_;
    transform_flip_z(vel_frd_, vel_flu_);

    // timestamp_.store(msg->timestamp);
}

void ControllerBase::callbackAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    rot_ned_frd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
    rot_ned_frd_.normalize();
    rot_nwu_flu_ = ROT_NWU_NED * rot_ned_frd_ * ROT_FRD_FLU;

    // timestamp_.store(msg->timestamp);
}

void ControllerBase::callbackAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
    angvel_frd_(0) = msg->xyz;
    angvel_frd_(1) = msg->xyz;
    angvel_frd_(2) = msg->xyz;
    transform_flip_z(angvel_frd_, angvel_flu_);

    // timestamp_.store(msg->timestamp);
}

void ControllerBase::callbackTimeSync(const px4_msgs::msg::Timesync::UniquePtr msg) {
    // timestamp_.store(msg->timestamp);
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerBase>());
    rclcpp::shutdown();
    return 0;
}
