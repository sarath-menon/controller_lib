#pragma once
#include "basic_axis_controller.h"
#include "math_helper.h"
#include "pid.h"

namespace horizontal_controllers {

class BasicHorizontalController {

protected:
  float dt_ = 0;

  float x_cmd_ = 0;
  float y_cmd_ = 0;

public:
  axis_controllers::BasicAxisController x_axis_ctrl;
  axis_controllers::BasicAxisController y_axis_ctrl;

public:
  void controller(const float x_target, const float y_target,
                  const float x_current, const float y_current);

public:
  /// Getter function
  const float x_cmd() const { return x_cmd_; }

  /// Getter function
  const float y_cmd() const { return y_cmd_; }

  /// Getter function
  const float dt() const { return dt_; }

  // Set controller rate
  void set_rate(float dt) {
    dt_ = dt;
    x_axis_ctrl.set_rate(dt);
    y_axis_ctrl.set_rate(dt);
  }
};

} // namespace horizontal_controllers