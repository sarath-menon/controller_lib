#pragma once
#include "math_helper.h"
#include "pid.h"

#include "safety_checks.h"
#include <string>
#include <yaml-cpp/yaml.h>

class Cascaded {

protected:
  // Feedforward thrust
  constexpr static float ff_thrust = 9.81;

  // Timescales
  int position_controller_rate = 0;
  int attitude_controller_rate = 0;
  int angular_velocity_controller_rate = 0;

  float position_controller_dt = 0;
  float attitude_controller_dt = 0;
  float angular_velocity_controller_dt = 0;

public:
  // Positon controllers
  float cascaded_controller();

public:
  // To load timescales from yaml
  void set_timescales(std::string path);
};