#pragma once
#include "cascaded.h"
#include "decoupled_pid.h"
#include "math_helper.h"
#include "quadcopter_properties.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

class BasicPidCascaded : public Cascaded, public QuadProperties {

private:
  // x position controller gains
  AxesPid y_pos;
  AxesPid z_pos;
  AxesPid roll_angle;

private:
  matrix::Vector<float, 4> thrust_torque_cmd;
  constexpr static float ff_thrust = 9.81;

public:
  // Positon controllers
  matrix::Vector<float, 4>
  cascaded_controller(const float position[3], const float orientation_euler[3],
                      const matrix::Vector<float, 3> position_target);

  // To load gain vaules from yaml file
  void set_gains(std::string path);

  // Set quadcopter state
  void set_state(const float position[3], const float orientation_euler[3]);

  // Set target
  void set_target(const float position_target[3]);
};