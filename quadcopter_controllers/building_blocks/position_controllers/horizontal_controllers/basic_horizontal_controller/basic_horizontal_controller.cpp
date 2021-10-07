#include "basic_horizontal_controller.h"
#include "pid.h"

namespace horizontal_controllers {

void BasicHorizontalController::controller(const float x_target,
                                           const float y_target,
                                           const float x_current,
                                           const float y_current) {

  x_cmd_ = x_axis_ctrl.controller(x_target, x_current);
  y_cmd_ = y_axis_ctrl.controller(y_target, y_current);
}
} // namespace horizontal_controllers