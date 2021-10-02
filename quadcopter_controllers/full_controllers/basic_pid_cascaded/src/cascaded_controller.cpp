#include "basic_pid_cascaded.h"

void BasicPidCascaded::cascaded_controller() {

  // Outer loop
  const float z_pos_error = position_target_[2] - position_[2];
  thrust_command = z_pos.controller(z_pos_error, ff_thrust);

  const float x_pos_error = position_target_[0] - position_[0];
  roll_angle_command = x_pos.controller(x_pos_error);

  // Inner loop
  const float roll_angle_error = roll_angle_command - orientation_euler_[2];
  torque_command = roll_angle.controller(roll_angle_error);
};
