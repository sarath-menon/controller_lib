#pragma once
#include "basic_horizontal_controller.h"
#include "basic_vertical_controller.h"
#include "cascaded.h"
#include "math_helper.h"
#include "quadcopter_properties.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

class BasicPidCascaded : public Cascaded, public QuadProperties {

public:
  BasicVerticalController vertical_ctrl;
  BasicHorizontalController horizontal_ctrl;

private:
  // Simulation timestep
  float dt = 0;
  matrix::Vector<float, 4> thrust_torque_cmd;

public:
  // Cascaded controller
  matrix::Vector<float, 4>
  cascaded_controller(const float position[3], const float orientation_euler[3],
                      const matrix::Vector<float, 3> position_target);

  // // To load gain vaules from yaml file
  // void set_gains(std::string path);

  // // To load quadcopter properties from yaml file
  // void set_quad_properties(std::string path);

  // // To load timescales from yaml
  // void set_timescales(std::string path);
};