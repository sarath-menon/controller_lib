#include "basic_pid_cascaded.h"

namespace controllers_2d {

matrix::Vector<float, 4> BasicPidCascaded::cascaded_controller(
    const std::array<double, 3> position,
    const std::array<double, 3> orientation_euler,
    const matrix::Vector<float, 3> position_target) {

  // Outer loop
  thrust_torque_cmd(0) = z_position_controller(position_target(2), position[2]);

  roll_angle_command = y_position_controller(position_target(1), position[1]);

  // Inner loop
  thrust_torque_cmd(1) =
      roll_angle_controller(roll_angle_command, -orientation_euler[0]);

  return thrust_torque_cmd;
};

matrix::Vector<float, 4> BasicPidCascaded::cascaded_controller(
    const msgs::Pose pose, const matrix::Vector<float, 3> position_target) {

  // Outer loop
  thrust_torque_cmd(0) =
      z_position_controller(position_target(2), pose.position.z);

  roll_angle_command =
      y_position_controller(position_target(1), pose.position.y);

  // Inner loop
  thrust_torque_cmd(1) =
      roll_angle_controller(roll_angle_command, -pose.orientation_euler.roll);

  return thrust_torque_cmd;
};

} // namespace controllers_2d