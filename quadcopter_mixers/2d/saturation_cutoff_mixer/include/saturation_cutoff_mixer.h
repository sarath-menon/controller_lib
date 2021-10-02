#pragma once
#include "math_helper.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

class QuadcopterMixer {

private:
  // Quadcopter Properties
  float arm_length = 0;
  float k_f = 0;
  float motor_thrust_max = 0;
  float motor_thrust_min = 0;

public:
  // Mixer
  void motor_mixer(float motor_commands[4], const float thrust_command,
                   const float torque_command);
  // Mixer
  matrix::Vector<float, 4> motor_mixer(const float thrust_command,
                                       const float torque_command);

public:
  // To load quadcopter properties from yaml file
  void set_quad_properties(std::string path);
};