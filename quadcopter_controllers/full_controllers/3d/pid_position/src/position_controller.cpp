#include "pid_position.h"

namespace controllers_3d {

cpp_msg::AttitudeCommand &
PositionPid::position_controller(const cpp_msg::Pose &pose,
                                 const cpp_msg::QuadPositionCmd &pos_setpoint) {

  // Outer loop
  attitude_cmd.thrust =
      z_position_controller(pos_setpoint.position.z, pose.position.z);

  attitude_cmd.roll =
      y_position_controller(pos_setpoint.position.y, pose.position.y);

  attitude_cmd.pitch =
      x_position_controller(pos_setpoint.position.x, pose.position.x);

  return attitude_cmd;
};

} // namespace controllers_3d