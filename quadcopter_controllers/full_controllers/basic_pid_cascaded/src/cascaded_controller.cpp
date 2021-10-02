#include "basic_pid_cascaded.h"

matrix::Vector<float, 4> BasicPidCascaded::cascaded_controller(
    const float position[3], const float orientation_euler[3],
    const matrix::Vector<float, 3> position_target) {

  // Outer loop
  const float z_pos_error = position_target(2) - position[2];
  thrust_torque_cmd(0) = z_pos.controller(z_pos_error, ff_thrust);

  const float y_pos_error = position_target(1) - position[1];
  const float roll_angle_command = y_pos.controller(y_pos_error);

  // Inner loop
  const float roll_angle_error = roll_angle_command - orientation_euler[2];
  thrust_torque_cmd(1) = roll_angle.controller(roll_angle_error);

  return thrust_torque_cmd;
};
