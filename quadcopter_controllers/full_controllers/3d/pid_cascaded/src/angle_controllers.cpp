#include "pid.h"
#include "pid_cascaded.h"

namespace controllers_3d {

float BasicPidCascaded::roll_angle_controller(const float roll_angle_target,
                                              const float roll_angle_now) {

  // x position pid variables
  static float e_i__roll = 0;
  static float e_d__roll = 0;
  static float e_prev__roll = 0;

  // Compute error
  const float error = roll_angle_target - roll_angle_now;
  // std::cout << "Roll angle error:" << error << '\n';

  // Compute control input
  float roll_torque_command =
      basic_controllers::pid(error, k_p__roll, k_i__roll, k_d__roll, dt,
                             e_i__roll, e_d__roll, e_prev__roll);

  // Limit roll angle to near zero to respect linearization
  roll_torque_command = math_helper::limit(roll_torque_command, roll_torque_max,
                                           -roll_torque_max);

  return roll_torque_command;
};

float BasicPidCascaded::pitch_angle_controller(const float pitch_angle_target,
                                               const float pitch_angle_now) {

  // x position pid variables
  static float e_i__pitch = 0;
  static float e_d__pitch = 0;
  static float e_prev__pitch = 0;

  // Compute error
  const float error = pitch_angle_target - pitch_angle_now;
  // std::cout << "pitch angle error:" << error << '\n';

  // Compute control input
  float pitch_torque_command =
      basic_controllers::pid(error, k_p__pitch, k_i__pitch, k_d__pitch, dt,
                             e_i__pitch, e_d__pitch, e_prev__pitch);

  // Limit roll angle to near zero to respect linearization
  pitch_torque_command = math_helper::limit(
      pitch_torque_command, pitch_torque_max, -pitch_torque_max);

  return pitch_torque_command;
};

} // namespace controllers_3d