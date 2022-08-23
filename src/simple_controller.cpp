#include "asl_flight2/simple_controller.h"

namespace asl {

using namespace std::chrono_literals;

SimpleController::SimpleController(const std::string& node_name)
    : ControllerBase(node_name),
      position_setpoint_(Eigen::Vector3d::Zero()),
      yaw_setpoint_(0),
      publish_loop_timer_(create_wall_timer(
        1ms, std::bind(&SimpleController::SetpointPublishLoop, this))) {

}

void SimpleController::SetpointPublishLoop() {
  SetPosition(position_setpoint_, yaw_setpoint_);
}

} // namespace asl
