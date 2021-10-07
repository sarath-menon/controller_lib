#include "basic_vertical_controller.h"
#include "pid.h"

namespace vertical_controllers {

void BasicVerticalController::controller(const float z_target,
                                         const float z_current) {

  z_cmd_ = z_axis_ctrl.controller(z_target, z_current, ff_thrust);
}
} // namespace vertical_controllers