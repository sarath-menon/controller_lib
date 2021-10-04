#pragma once
#include "math_helper.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

class QuadcopterMixer {

private:
  // Quadcopter Properties
  float arm_length = 0.171;

  float k_f = 6.11e-8;
  float k_t = 1.5e-9;

  // Individual motor thrust max, min
  float motor_thrust_max = 0;
  float motor_thrust_min = 0;

  // Quadcopter net thrust max, min
  float thrust_max = 27;
  float thrust_min = 7;

public:
  // Mixer
  // void motor_mixer(float motor_commands[4], const float thrust_command,
  //                  const float torque_command);
  // Mixer

  // Initialize mixer matrix
  QuadcopterMixer();

  // Mixer
  matrix::Vector<float, 4>
  motor_mixer(const matrix::Vector<float, 4> thrust_torque_command);

  // Vector of motor commands
  matrix::Vector<float, 4> motor_commands;

  // Mixer Matrix
  matrix::SquareMatrix<float, 4> mixer_matrix_;

public:
  // To load quadcopter properties from yaml file
  void set_quad_properties(std::string path);
};