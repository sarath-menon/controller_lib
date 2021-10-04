#pragma once
#include "cascaded.h"
#include "math_helper.h"
#include "quadcopter_properties.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

class BasicPidCascaded : public Cascaded, public QuadProperties {

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

private:
  // Simulation timestep
  float dt = 0;

  matrix::Vector<float, 4> thrust_torque_cmd;
  constexpr static float ff_thrust = 9.81;

public:
  // Cascaded controller
  matrix::Vector<float, 4>
  cascaded_controller(const float position[3], const float orientation_euler[3],
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

  // Set quadcopter state
  void set_state(const float position[3], const float orientation_euler[3]);

  // Set target
  void set_target(const float position_target[3]);
};