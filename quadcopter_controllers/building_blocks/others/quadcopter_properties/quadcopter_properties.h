#pragma once

#include "safety_checks.h"
#include <string>
#include <yaml-cpp/yaml.h>

class QuadProperties {

protected:
  // Quadcopter properties
  float arm_length = 0;

  float propeller_thrust_max = 0;
  float propeller_thrust_min = 0;

  float net_thrust_max = 0;
  float net_thrust_min = 0;

  float roll_angle_max = 0;
  float pitch_angle_max = 0;
  float yaw_angle_max = 0;

  float roll_torque_max = 0;
  float pitch_torque_max = 0;
  float yaw_torque_max = 0;

public:
  // To load quadcopter properties from yaml file
  void set_quadcopter_properties(std::string path);
};