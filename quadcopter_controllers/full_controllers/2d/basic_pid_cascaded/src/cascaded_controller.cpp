#include "basic_pid_cascaded.h"

namespace controllers_2d {

cpp_msg::ThrustTorqueCommand &
BasicPidCascaded::cascaded_controller(const cpp_msg::Pose &pose,
                                      const cpp_msg::Pose &pose_setpoint) {

  // Outer loop
  thrust_torque_cmd.thrust =
      z_position_controller(pose_setpoint.position.z, pose.position.z);

  roll_angle_command =
      y_position_controller(pose_setpoint.position.y, pose.position.y);

  // Inner loop
  thrust_torque_cmd.roll_torque =
      roll_angle_controller(roll_angle_command, pose.orientation_euler.roll);

  return thrust_torque_cmd;
};

} // namespace controllers_2d