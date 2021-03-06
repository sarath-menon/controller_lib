#include "pid_cascaded.h"

namespace controllers_3d {

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

cpp_msg::ThrustTorqueCommand &BasicPidCascaded::cascaded_controller(
    const cpp_msg::Pose &pose, const cpp_msg::QuadPositionCmd &pos_setpoint) {

  // Outer loop
  thrust_torque_cmd.thrust =
      z_position_controller(pos_setpoint.position.z, pose.position.z);

  roll_angle_command =
      y_position_controller(pos_setpoint.position.y, pose.position.y);

  pitch_angle_command =
      x_position_controller(pos_setpoint.position.x, pose.position.x);

  // Inner loop
  thrust_torque_cmd.roll_torque =
      roll_angle_controller(roll_angle_command, pose.orientation_euler.roll);

  thrust_torque_cmd.pitch_torque =
      pitch_angle_controller(pitch_angle_command, pose.orientation_euler.pitch);

  //   std::cout << "Cmd: " << thrust_torque_cmd.thrust << '\t'
  //             << thrust_torque_cmd.roll_torque << '\t'
  //             << thrust_torque_cmd.pitch_torque << '\t'
  //             << thrust_torque_cmd.yaw_torque << std::endl;

  return thrust_torque_cmd;
};

} // namespace controllers_3d