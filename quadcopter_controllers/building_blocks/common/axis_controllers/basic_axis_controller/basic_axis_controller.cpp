#include "basic_axis_controller.h"
#include "pid.h"

namespace axis_controllers {

// Positon controllers
float BasicAxisController::controller(const float target,
                                      const float current_val,
                                      const float feedforward) {

  // Compute error
  const float error = target - current_val;

  // x position pid variables
  static float e_i = 0;
  static float e_d = 0;
  static float e_prev = 0;

  // Compute control input
  float control_command =
      basic_controllers::pid(error, k_p, k_i, k_d, dt_, e_i, e_d, e_prev);

  // Limit roll angle to near zero to respect linearization
  control_command = math_helper::limit(control_command + feedforward, max, min);

  return control_command;
}
} // namespace axis_controllers
