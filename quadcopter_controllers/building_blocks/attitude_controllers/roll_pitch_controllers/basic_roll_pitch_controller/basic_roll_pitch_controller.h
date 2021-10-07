#pragma once
#include "basic_axis_controller.h"
#include "math_helper.h"
#include "pid.h"

namespace roll_pitch_controllers {

class BasicRollPitchController {

protected:
  float dt_ = 0;

  float roll_cmd_ = 0;
  float pitch_cmd_ = 0;

public:
  axis_controllers::BasicAxisController roll_ctrl;
  axis_controllers::BasicAxisController pitch_ctrl;

public:
  void controller(const float roll_target, const float pitch_target,
                  const float roll_current, const float pitch_current);

public:
  /// Getter function
  const float roll_cmd() const { return roll_cmd_; }

  /// Getter function
  const float pitch_cmd() const { return pitch_cmd_; }

  /// Getter function
  const float dt() const { return dt_; }

  // Set controller rate
  void set_rate(float dt) {
    dt_ = dt;
    roll_ctrl.set_rate(dt);
    roll_ctrl.set_rate(dt);
  }
};

} // namespace roll_pitch_controllers