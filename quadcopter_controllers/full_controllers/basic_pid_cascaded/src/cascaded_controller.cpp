#include "basic_pid_cascaded.h"

matrix::Vector<float, 4> BasicPidCascaded::cascaded_controller(
    const float position[3], const float orientation_euler[3],
    const matrix::Vector<float, 3> position_target) {

  // Outer loop
  thrust_torque_cmd(0) = z_position_controller(position_target(2), position[2]);

  roll_angle_command = y_position_controller(position_target(1), position[1]);

  // Inner loop
  thrust_torque_cmd(1) =
      roll_angle_controller(roll_angle_command, -orientation_euler[0]);

  return thrust_torque_cmd;
};
