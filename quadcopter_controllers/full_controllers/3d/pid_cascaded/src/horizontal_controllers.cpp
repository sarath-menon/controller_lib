#include "pid.h"
#include "pid_cascaded.h"

namespace controllers_3d {
float BasicPidCascaded::y_position_controller(const float y_position_target,
                                              const float y_position_now) {

  // x position pid variables
  static float e_i__y = 0;
  static float e_d__y = 0;
  static float e_prev__y = 0;

  // Compute error
  const float error = y_position_target - y_position_now;

  // Compute control input
  float roll_angle_command = basic_controllers::pid(
      error, k_p__y, k_i__y, k_d__y, dt, e_i__y, e_d__y, e_prev__y);

  // Limit roll angle to near zero to respect linearization
  roll_angle_command =
      -math_helper::limit(roll_angle_command, roll_angle_max, -roll_angle_max);

  return roll_angle_command;
};

float BasicPidCascaded::x_position_controller(const float x_position_target,
                                              const float x_position_now) {

  // x position pid variables
  static float e_i__x = 0;
  static float e_d__x = 0;
  static float e_prev__x = 0;

  // Compute error
  const float error = x_position_target - x_position_now;

  // Compute control input
  float pitch_angle_command = basic_controllers::pid(
      error, k_p__x, k_i__x, k_d__x, dt, e_i__x, e_d__x, e_prev__x);

  // Limit roll angle to near zero to respect linearization
  pitch_angle_command = math_helper::limit(pitch_angle_command, pitch_angle_max,
                                           -pitch_angle_max);

  return pitch_angle_command;
};

} // namespace controllers_3d