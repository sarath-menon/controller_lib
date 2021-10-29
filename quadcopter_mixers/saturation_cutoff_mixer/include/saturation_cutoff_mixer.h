#pragma once
#include "math_helper.h"
#include "quadcopter_msgs/msgs/QuadMotorCommand.h"
#include "quadcopter_msgs/msgs/ThrustTorqueCommand.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

class QuadcopterMixer {

private:
  // Quadcopter Properties
  float arm_length{};

  // Motor constants: thrust-speed^2 and torque-speed^2
  float k_f{};
  float k_t{};

  // Individual motor thrust max, min
  float motor_thrust_max{};
  float motor_thrust_min{};

  // Quadcopter net thrust max, min
  float thrust_max{};
  float thrust_min{};

  // Vector of motor commands
  cpp_msg::QuadMotorCommand motor_cmd{};

public:
  // Mixer
  cpp_msg::QuadMotorCommand &
  motor_mixer(const cpp_msg::ThrustTorqueCommand &thrust_torque_cmd);

  // Mixer Matrix
  matrix::SquareMatrix<float, 4> mixer_matrix_;

public:
  // To load quadcopter properties from yaml file
  void set_quad_properties(std::string path);

  // Set mixer matrix x
  void set_mixer_matrix_x();

  // Set mixer matrix plus
  void set_mixer_matrix_plus();
};