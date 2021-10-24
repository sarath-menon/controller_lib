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

  //   std::cout << "Cmd: " << thrust_torque_cmd.thrust << '\t'
  //             << thrust_torque_cmd.roll_torque << '\t'
  //             << thrust_torque_cmd.pitch_torque << '\t'
  //             << thrust_torque_cmd.yaw_torque << std::endl;

  return thrust_torque_cmd;
};

} // namespace controllers_3d