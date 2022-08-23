#pragma once

#include "asl_flight2/controller_base.h"

namespace asl {

class SimpleController : public ControllerBase {
 public:
  explicit SimpleController(const std::string& node_name);

 protected:
  mutable Eigen::Vector3d position_setpoint_;
  mutable double yaw_setpoint_;

  void TakeOff(const double& height, const std::chrono::seconds& duration);
  void Land(const std::chrono::seconds& duration);

 private:
  const rclcpp::TimerBase::SharedPtr publish_loop_timer_;

  void SetpointPublishLoop();
};

} // namespace asl
