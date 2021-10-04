#include "basic_pid_cascaded.h"
#include "pid.h"

float BasicPidCascaded::y_position_controller(const float y_position_target,
                                              const float y_position_now) {

  // x position pid variables
  static float e_i__y = 0;
  static float e_d__y = 0;
  static float e_prev__y = 0;

  // Compute error
  const float error = y_position_target - y_position_now;

  // Compute control input
  float roll_angle_command =
      basic_controllers::pid(error, k_p__y, k_i__y, k_d__y, dt, e_i__y, e_d__y,
                             e_prev__y) /
      9.81;

  // Limit roll angle to near zero to respect linearization
  roll_angle_command =
      math_helper::limit(roll_angle_command, roll_angle_max, -roll_angle_max);

  return roll_angle_command;
};
