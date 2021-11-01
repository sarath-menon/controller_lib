#pragma once
#include "geometry_msgs/msgs/Pose.h"
#include "math_helper.h"
#include "quadcopter_msgs/msgs/QuadPositionCmd.h"
#include "quadcopter_msgs/msgs/ThrustTorqueCommand.h"
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace controllers_3d {

class BasicPidCascaded {

private:
  // y position controller gains
  float k_p__y{}; // [constant]
  float k_i__y{}; // [constant]
  float k_d__y{}; // [constant]

  // x position controller gains
  float k_p__x{}; // [constant]
  float k_i__x{}; // [constant]
  float k_d__x{}; // [constant]

  // z position controller gains
  float k_p__z{}; // [constant]
  float k_i__z{}; // [constant]
  float k_d__z{}; // [constant]

  // roll angle controller parameters
  float k_p__roll{}; // [constant]
  float k_i__roll{}; // [constant]
  float k_d__roll{}; // [constant]

  // pitch angle controller parameters
  float k_p__pitch{}; // [constant]
  float k_i__pitch{}; // [constant]
  float k_d__pitch{}; // [constant]

private:
  // Quadcopter properties
  float mass{};
  float arm_length{};

  float propeller_thrust_max{};
  float propeller_thrust_min{};

  float net_thrust_max{};
  float net_thrust_min{};

  float roll_angle_max{};
  float roll_torque_max{};
  float pitch_angle_max{};
  float pitch_torque_max{};

  // Timescales
  int position_loop_rate{};
  int attitude_loop_rate{};

  float position_dt{};
  float attitude_dt{};

  float dt{};

  // Control commands
  cpp_msg::ThrustTorqueCommand thrust_torque_cmd{};

  float roll_angle_command{};
  float pitch_angle_command{};

  // Feedforward thrust
  constexpr static float ff_thrust = 9.81;

public:
  // Cascaded controller
  cpp_msg::ThrustTorqueCommand &
  cascaded_controller(const cpp_msg::Pose &pose,
                      const cpp_msg::Pose &pose_setpoint);

  // Cascaded controller new
  cpp_msg::ThrustTorqueCommand &
  cascaded_controller(const cpp_msg::Pose &pose,
                      const cpp_msg::QuadPositionCmd &pos_setpoint);

  // Horizontal controller
  float y_position_controller(const float y_position_target,
                              const float y_position_now);
  float x_position_controller(const float x_position_target,
                              const float x_position_now);

  // Vertical controller
  float z_position_controller(const float z_position_target,
                              const float z_position_now);

  // Angle controllers
  float roll_angle_controller(const float roll_angle_target,
                              const float roll_angle_now);
  float pitch_angle_controller(const float pitch_angle_target,
                               const float pitch_angle_now);

  // To load gain vaules from yaml file
  void set_gains(const std::string &path);

  // To load quadcopter properties from yaml file
  void set_quad_properties(const std::string &path);

  // To load timescales from yaml
  void set_timescales(const std::string &path);
};

} // namespace controllers_3d