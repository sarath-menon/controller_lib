#pragma once
#include "math_helper.h"
#include "pid.h"

namespace axis_controllers {

class BasicAxisController {

protected:
  // PID gains
  float k_p = 0; // [constant]
  float k_i = 0; // [constant]
  float k_d = 0; // [constant]

  // limits
  float min = 0;
  float max = 0;

  // controller ate
  float dt_ = 0;

public:
  float controller(const float target, const float current_val,
                   const float feedforward = 0);

public:
  // Set k_p gain
  void set_k_p(float val) { k_p = val; }
  // Set k_i gain
  void set_k_i(float val) { k_i = val; }
  // Set k_p gain
  void set_k_d(float val) { k_d = val; }
  // Set controller rate
  void set_rate(float dt) { dt_ = dt; }
};

} // namespace axis_controllers