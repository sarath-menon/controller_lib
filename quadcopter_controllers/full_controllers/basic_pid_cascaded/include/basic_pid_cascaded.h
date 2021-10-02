#pragma once
#include "cascaded.h"
#include "decoupled_pid.h"
#include "math_helper.h"
#include "quadcopter_properties.h"
#include "safety_checks.h"
#include <string>
#include <yaml-cpp/yaml.h>

class BasicPidCascaded : public Cascaded, public QuadProperties {

private:
  // x position controller gains
  AxesPid x_pos;
  AxesPid z_pos;
  AxesPid roll_angle;

private:
  float thrust_command = 0;
  float roll_angle_command = 0;
  float torque_command = 0;

  constexpr static float ff_thrust = 9.81;

public:
  // State
  float position_[3] = {0, 0, 0};
  float orientation_euler_[3] = {0, 0, 0};

  // target
  float position_target_[3] = {0, 0, 0};

public:
  // Positon controllers
  void cascaded_controller();

  // To load gain vaules from yaml file
  void set_gains(std::string path);

  // Set quadcopter state
  void set_state(const float position[3], const float orientation_euler[3]);

  // Set target
  void set_target(const float position_target[3]);
};