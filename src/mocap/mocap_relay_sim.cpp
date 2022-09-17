#include <functional>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

#include "asl_flight2/mocap_relay.hpp"

class MocapRelaySim : public asl::MocapRelay {
 public:
  explicit MocapRelaySim(const std::string& node_name = "mocap_relay_sim")
    : asl::MocapRelay(node_name),
      model_name_(this->declare_parameter("model_name", "iris")),
      gz_node_(new gazebo::transport::Node()) {
    gz_node_->Init();
    gz_sub_ = gz_node_->Subscribe("~/pose/info", &MocapRelaySim::PoseCallback, this);
  }

 private:
  const std::string model_name_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::SubscriberPtr gz_sub_;

  void PoseCallback(ConstPosesStampedPtr& msg) {
    // use ROS2 time
    const rclcpp::Time now = this->now();

    for (int i = 0; i < msg->pose_size(); ++i) {
      const auto pose_tmp = msg->pose(i);
      if (pose_tmp.has_name() && pose_tmp.name() == model_name_) {
        const tf2::Transform world_nwu_T_body_nwu(
          tf2::Quaternion(pose_tmp.orientation().x(),
                          pose_tmp.orientation().y(),
                          pose_tmp.orientation().z(),
                          pose_tmp.orientation().w()),
          tf2::Vector3(pose_tmp.position().x(),
                       pose_tmp.position().y(),
                       pose_tmp.position().z())
        );
        const tf2::Transform nwu_T_neu(tf2::Quaternion(1, 0, 0, 0));
        const tf2::Transform world_nwu_T_body_neu = world_nwu_T_body_nwu * nwu_T_neu;

        geometry_msgs::msg::PoseStamped pose_stamped{};
        tf2::toMsg(world_nwu_T_body_neu, pose_stamped.pose);

        pose_stamped.header.stamp = now;
        pose_stamped.header.frame_id = "world_nwu";

        this->PublishPose(pose_stamped);
      }
    }
  }
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  gazebo::client::setup(argc, argv);

  rclcpp::spin(std::make_shared<MocapRelaySim>());

  gazebo::client::shutdown();
  rclcpp::shutdown();
}