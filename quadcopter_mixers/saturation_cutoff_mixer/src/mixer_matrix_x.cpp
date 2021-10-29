#include "saturation_cutoff_mixer.h"
#include <iostream>

void QuadcopterMixer::set_mixer_matrix_x() {

  const float moment_arm = arm_length / std::sqrtf(2);
  const float moment_arm_inv = 1.0 / moment_arm;
  const float k_t_inv = 1.0 / k_t;

  mixer_matrix_(0, 0) = 1.0;
  mixer_matrix_(0, 1) = moment_arm_inv;
  mixer_matrix_(0, 2) = -moment_arm_inv;
  mixer_matrix_(0, 3) = k_t_inv;

  mixer_matrix_(1, 0) = 1.0;
  mixer_matrix_(1, 1) = -moment_arm_inv;
  mixer_matrix_(1, 2) = -moment_arm_inv;
  mixer_matrix_(1, 3) = -k_t_inv;

  mixer_matrix_(2, 0) = 1.0;
  mixer_matrix_(2, 1) = -moment_arm_inv;
  mixer_matrix_(2, 2) = moment_arm_inv;
  mixer_matrix_(2, 3) = k_t_inv;

  mixer_matrix_(3, 0) = 1.0;
  mixer_matrix_(3, 1) = moment_arm_inv;
  mixer_matrix_(3, 2) = moment_arm_inv;
  mixer_matrix_(3, 3) = -k_t_inv;

  // Multiplt matrix by 0.25
  mixer_matrix_ *= 0.25;
}
