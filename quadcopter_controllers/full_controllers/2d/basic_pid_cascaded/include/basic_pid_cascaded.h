#pragma once
#include "math_helper.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace controllers_2d {

class BasicPidCascaded {

private:
  // y position controller gains
  float k_p__y = 0; // [constant]
  float k_i__y = 0; // [constant]
  float k_d__y = 0; // [constant]
  // z position controller gains
  float k_p__z = 0; // [constant]
  float k_i__z = 0; // [constant]
  float k_d__z = 0; // [constant]
  // roll angle controller parameters
  float k_p__roll = 0; // [constant]
  float k_i__roll = 0; // [constant]
  float k_d__roll = 0; // [constant]

private:
  // Quadcopter properties
  float arm_length = 0;

  float propeller_thrust_max = 0;
  float propeller_thrust_min = 0;

  float net_thrust_max = 0;
  float net_thrust_min = 0;

  float roll_angle_max = 0;
  float roll_torque_max = 0;

  // Timescales
  int position_loop_rate = 0;
  int attitude_loop_rate = 0;

  float position_dt = 0;
  float attitude_dt = 0;

  float dt = 0;

  // Control commands
  matrix::Vector<float, 4> thrust_torque_cmd;
  float roll_angle_command;

  // Feedforward thrust
  constexpr static float ff_thrust = 9.81;

public:
  // Cascaded controller
  matrix::Vector<float, 4>
  cascaded_controller(const std::array<double, 3> position,
                      const std::array<double, 3> orientation_euler,
                      const matrix::Vector<float, 3> position_target);

  // Decoupled controllers
  float y_position_controller(const float y_position_target,
                              const float y_position_now);

  float z_position_controller(const float z_position_target,
                              const float z_position_now);

  float roll_angle_controller(const float roll_angle_target,
                              const float roll_angle_now);

  // To load gain vaules from yaml file
  void set_gains(std::string path);

  // To load quadcopter properties from yaml file
  void set_quad_properties(std::string path);

  // To load timescales from yaml
  void set_timescales(std::string path);
};

} // namespace controllers_2d