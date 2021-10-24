#pragma once
#include "geometry_msgs/msgs/Pose.h"
#include "math_helper.h"
#include "quadcopter_msgs/msgs/ThrustTorqueCommand.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace controllers_2d {

class BasicPidCascaded {

private:
  // y position controller gains
  float k_p__y{}; // [constant]
  float k_i__y{}; // [constant]
  float k_d__y{}; // [constant]
  // z position controller gains
  float k_p__z{}; // [constant]
  float k_i__z{}; // [constant]
  float k_d__z{}; // [constant]
  // roll angle controller parameters
  float k_p__roll{}; // [constant]
  float k_i__roll{}; // [constant]
  float k_d__roll{}; // [constant]

private:
  // Quadcopter properties
  float arm_length{};

  float propeller_thrust_max{};
  float propeller_thrust_min{};

  float net_thrust_max{};
  float net_thrust_min{};

  float roll_angle_max{};
  float roll_torque_max{};

  // Timescales
  int position_loop_rate{};
  int attitude_loop_rate{};

  float position_dt{};
  float attitude_dt{};

  float dt{};

  // Control commands
  cpp_msg::ThrustTorqueCommand thrust_torque_cmd{};
  float roll_angle_command{};

  // Feedforward thrust
  constexpr static float ff_thrust = 9.81;

public:
  // Cascaded controller
  cpp_msg::ThrustTorqueCommand &
  cascaded_controller(const cpp_msg::Pose &pose,
                      const cpp_msg::Pose &pose_setpoint);

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