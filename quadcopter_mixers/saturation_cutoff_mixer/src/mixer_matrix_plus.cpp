#include "saturation_cutoff_mixer.h"
#include <iostream>

void QuadcopterMixer::set_mixer_matrix_plus() {
  mixer_matrix_(0, 0) = 0.25;
  mixer_matrix_(0, 1) = 0.0;
  mixer_matrix_(0, 2) = 0.5 / arm_length;
  mixer_matrix_(0, 3) = 0.25 / k_t;

  mixer_matrix_(1, 0) = 0.25;
  mixer_matrix_(1, 1) = 0.5 / arm_length;
  mixer_matrix_(1, 2) = 0.0;
  mixer_matrix_(1, 3) = -0.25 / k_t;

  mixer_matrix_(2, 0) = 0.25;
  mixer_matrix_(2, 1) = 0.0;
  mixer_matrix_(2, 2) = -0.5 / arm_length;
  mixer_matrix_(2, 3) = 0.25 / k_t;

  mixer_matrix_(3, 0) = 0.25;
  mixer_matrix_(3, 1) = -0.5 / arm_length;
  mixer_matrix_(3, 2) = 0.0;
  mixer_matrix_(3, 3) = -0.25 / k_t;
}
