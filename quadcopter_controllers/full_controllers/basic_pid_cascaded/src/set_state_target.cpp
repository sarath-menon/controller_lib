#include "basic_pid_cascaded.h"

void BasicPidCascaded::set_state(const float position[3],
                                 const float orientation_euler[3]) {
  position_[0] = position[0];
  position_[1] = position[1];
  position_[2] = position[2];

  orientation_euler_[0] = orientation_euler[0];
  orientation_euler_[1] = orientation_euler[1];
  orientation_euler_[2] = orientation_euler[2];
}

void BasicPidCascaded::set_target(const float position_target[3]) {

  position_target_[0] = position_target[0];
  position_target_[1] = position_target[1];
  position_target_[2] = position_target[2];
}