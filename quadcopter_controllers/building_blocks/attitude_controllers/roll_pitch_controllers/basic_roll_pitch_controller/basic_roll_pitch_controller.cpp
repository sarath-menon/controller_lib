#include "basic_roll_pitch_controller.h"
#include "pid.h"

namespace roll_pitch_controllers {

void BasicRollPitchController::controller(const float roll_target,
                                          const float pitch_target,
                                          const float roll_current,
                                          const float pitch_current) {

  roll_cmd_ = roll_ctrl.controller(roll_target, roll_current);
  pitch_cmd_ = pitch_ctrl.controller(pitch_target, pitch_current);
}
} // namespace roll_pitch_controllers
