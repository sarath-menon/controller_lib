#include "basic_pid_cascaded.h"

namespace controllers_2d {

msgs::ThrustTorqueCommand BasicPidCascaded::cascaded_controller(
    const msgs::Pose &pose, const matrix::Vector<float, 3> &position_target) {

  // Outer loop
  thrust_torque_cmd.thrust =
      z_position_controller(position_target(2), pose.position.z);

  roll_angle_command =
      y_position_controller(position_target(1), pose.position.y);

  // Inner loop
  thrust_torque_cmd.roll_torque =
      roll_angle_controller(roll_angle_command, pose.orientation_euler.roll);

  return thrust_torque_cmd;
};

} // namespace controllers_2d