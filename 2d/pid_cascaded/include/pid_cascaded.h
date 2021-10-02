#pragma once
#include "math_helper.h"
#include "pid.h"
#include "safety_checks.h"
#include <string>
#include <yaml-cpp/yaml.h>

class PidCascadedController {

private:
  // x position controller gains
  float k_p__x = 0; // [constant]
  float k_i__x = 0; // [constant]
  float k_d__x = 0; // [constant]
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
  // roll angle controller parameters
  float k_p__pitch = 0; // [constant]
  float k_i__pitch = 0; // [constant]
  float k_d__pitch = 0; // [constant]

  // Simulation timestep
  float dt = 0;

  // Feedforward thrust
  constexpr static float ff_thrust = 9.81;

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

public:
  // Positon controllers
  float x_position_controller(const float x_position_target,
                              const float x_position_now);
  // float y_position_controller(const float y_position_target,
  //                             const float y_position_now,
  //                             const float pitch_angle_max,
  //                             const float pitch_angle_min, const float dt);
  float z_position_controller(const float z_position_target,
                              const float z_position_now);

  // Attitude controller
  float roll_angle_controller(const float roll_angle_target,
                              const float roll_angle_now);
  // float pitch_angle_controller(const float pitch_angle_target,
  //                              const float pitch_angle_now,
  //                              const float pitch_torque_max,
  //                              const float pitch_torque_min, const float dt);
  // float yaw_angle_controller(const float yaw_angle_target,
  //                            const float yaw_angle_now,
  //                            const float yaw_torque_max, const float
  //                            yaw_torque_min, const float dt);

public:
  // To load gain vaules from yaml file
  void set_gains(std::string path);
  // To load quadcopter properties from yaml file
  void set_quad_properties(std::string path);
  // To load timescales from yaml
  void set_timescales(std::string path);
};