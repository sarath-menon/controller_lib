#include "pid_cascaded.h"

namespace controllers_3d {

void BasicPidCascaded::set_gains(const std::string &path) {
  // Safety check, see if file exists
  safety_checks::yaml_file_check(path);

  // Load yaml file containing gains
  YAML::Node controller_yaml = YAML::LoadFile(path);

  // y position controller gains
  k_p__y = controller_yaml["k_p__y"].as<float>(); // [constant]
  k_i__y = controller_yaml["k_i__y"].as<float>(); // [constant]
  k_d__y = controller_yaml["k_d__y"].as<float>(); // [constant]

  // x position controller gains
  k_p__x = controller_yaml["k_p__x"].as<float>(); // [constant]
  k_i__x = controller_yaml["k_i__x"].as<float>(); // [constant]
  k_d__x = controller_yaml["k_d__x"].as<float>(); // [constant]

  // z position controller gains
  k_p__z = controller_yaml["k_p__z"].as<float>(); // [constant]
  k_i__z = controller_yaml["k_i__z"].as<float>(); // [constant]
  k_d__z = controller_yaml["k_d__z"].as<float>(); // [constant]

  // roll angle controller gains
  k_p__roll = controller_yaml["k_p__roll"].as<float>(); // [constant]
  k_i__roll = controller_yaml["k_i__roll"].as<float>(); // [constant]
  k_d__roll = controller_yaml["k_d__roll"].as<float>(); // [constant]

  // pitch angle controller gains
  k_p__pitch = controller_yaml["k_p__pitch"].as<float>(); // [constant]
  k_i__pitch = controller_yaml["k_i__pitch"].as<float>(); // [constant]
  k_d__pitch = controller_yaml["k_d__pitch"].as<float>(); // [constant]

  // simulation timestep
  dt = controller_yaml["dt"].as<float>(); // [constant]
};

void BasicPidCascaded::set_quad_properties(const std::string &path) {
  // Safety check, see if file exists
  safety_checks::yaml_file_check(path);

  // Load yaml file containing quadcopter properties
  YAML::Node quad_yaml = YAML::LoadFile(path);

  // Set quadcopter properties
  arm_length = quad_yaml["arm_length"].as<float>(); // [constant]

  propeller_thrust_max =
      quad_yaml["motor_thrust_max"].as<float>(); // [constant]
  propeller_thrust_min =
      quad_yaml["motor_thrust_min"].as<float>(); // [constant]

  net_thrust_max = propeller_thrust_max * 4;
  net_thrust_min = propeller_thrust_min * 4;

  roll_angle_max = quad_yaml["roll_angle_max"].as<float>(); // [constant]
  roll_torque_max = (propeller_thrust_max - propeller_thrust_min) * arm_length;

  pitch_angle_max = quad_yaml["pitch_angle_max"].as<float>(); // [constant]
  pitch_torque_max = roll_torque_max;
}

void BasicPidCascaded::set_timescales(const std::string &path) {
  // Load yaml file containing quadcopter properties
  YAML::Node timescale_yaml = YAML::LoadFile(path);

  // Load timescales from yaml file
  position_loop_rate = timescale_yaml["position_loop_rate"].as<float>();
  attitude_loop_rate = timescale_yaml["attitude_loop_rate"].as<float>();
  // Compute dt from timescale
  position_dt = 1 / position_loop_rate;
  attitude_dt = 1 / attitude_loop_rate;
}

} // namespace controllers_3d