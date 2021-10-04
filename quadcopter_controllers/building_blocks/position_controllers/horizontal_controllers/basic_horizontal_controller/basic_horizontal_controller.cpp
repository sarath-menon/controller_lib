#include "decoupled_pid.h"
#include "pid.h"

// Positon controllers
float BasicHorizontalController::controller(const float error,
                                            const float feedforward) {

  // x position pid variables
  static float e_i = 0;
  static float e_d = 0;
  static float e_prev = 0;

  // Compute control input
  float control_command =
      basic_controllers::pid(error, k_p, k_i, k_d, dt, e_i, e_d, e_prev);

  // Limit roll angle to near zero to respect linearization
  control_command = math_helper::limit(control_command + feedforward, max, min);

  return control_command;
}
