#pragma once
#include "basic_axis_controller.h"
#include "math_helper.h"
#include "pid.h"

namespace vertical_controllers {

class BasicVerticalController {
private:
  constexpr static float ff_thrust = 9.81;

protected:
  float dt_ = 0;
  float z_cmd_ = 0;

public:
  axis_controllers::BasicAxisController z_axis_ctrl;

public:
  void controller(const float z_target, const float z_current);

public:
  /// Getter function
  const float z_cmd() const { return z_cmd_; }

  /// Getter function
  const float dt() const { return dt_; }

  // Set controller rate
  void set_rate(float dt) {
    dt_ = dt;
    z_axis_ctrl.set_rate(dt);
  }
};

} // namespace vertical_controllers