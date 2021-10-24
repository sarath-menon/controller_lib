#include "pid.h"
#include "pid_cascaded.h"

namespace controllers_3d {

float BasicPidCascaded::z_position_controller(const float z_position_target,
                                              const float z_position_now) {

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

} // namespace controllers_3d