#pragma once

#include "safety_checks.h"
#include <string>
#include <yaml-cpp/yaml.h>

class QuadProperties {

protected:
  // Quadcopter properties
  float arm_length{};

  float propeller_thrust_max{};
  float propeller_thrust_min{};

  float net_thrust_max{};
  float net_thrust_min{};

  float roll_angle_max{};
  float pitch_angle_max{};
  float yaw_angle_max{};

  float roll_torque_max{};
  float pitch_torque_max{};
  float yaw_torque_max{};

public:
  // To load quadcopter properties from yaml file
  void set_quadcopter_properties(std::string path);
};