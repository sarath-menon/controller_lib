#include "pid_cascaded.h"
#include <iostream>

float PidCascadedController::x_position_controller(
    const float x_position_target, const float x_position_now) {

  // x position pid variables
  static float e_i__x = 0;
  static float e_d__x = 0;
  static float e_prev__x = 0;

  // Compute error
  const float error = x_position_target - x_position_now;

  // Compute control input
  float roll_angle_command =
      basic_controllers::pid(error, k_p__x, k_i__x, k_d__x, dt, e_i__x, e_d__x,
                             e_prev__x) /
      9.81;

  // Limit roll angle to near zero to respect linearization
  roll_angle_command =
      math_helper::limit(roll_angle_command, roll_angle_max, -roll_angle_max);

  return roll_angle_command;
};

float PidCascadedController::z_position_controller(
    const float z_position_target, const float z_position_now) {

  // y position pid variables
  static float e_i__z = 0;
  static float e_d__z = 0;
  static float e_prev__z = 0;

  // Compute error
  const float error = z_position_target - z_position_now;

  // Compute control input
  float thrust_command = basic_controllers::pid(error, k_p__z, k_i__z, k_d__z,
                                                dt, e_i__z, e_d__z, e_prev__z);

  // Limit roll angle to near zero to respect linearization
  thrust_command = math_helper::limit(ff_thrust + thrust_command,
                                      net_thrust_max, net_thrust_min);

  return thrust_command;
};

float PidCascadedController::roll_angle_controller(
    const float roll_angle_target, const float roll_angle_now) {

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