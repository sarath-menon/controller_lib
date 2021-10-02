#include "basic_pid_cascaded.h"

void BasicPidCascaded::set_gains(std::string path) {
  // Safety check, see if file exists
  safety_checks::yaml_file_check(path);

  // Load yaml file containing gains
  YAML::Node yaml = YAML::LoadFile(path);

  // x position controller gains
  y_pos.set_k_p(yaml["k_p__x"].as<float>()); // [constant]
  y_pos.set_k_i(yaml["k_i__x"].as<float>()); // [constant]
  y_pos.set_k_d(yaml["k_d__x"].as<float>()); // [constant]

  // z position controller gains
  z_pos.set_k_p(yaml["k_p__z"].as<float>()); // [constant]
  z_pos.set_k_i(yaml["k_i__z"].as<float>()); // [constant]
  z_pos.set_k_d(yaml["k_d__z"].as<float>()); // [constant]

  // roll angle controller gains
  roll_angle.set_k_p(yaml["k_p__roll"].as<float>()); // [constant]
  roll_angle.set_k_i(yaml["k_i__roll"].as<float>()); // [constant]
  roll_angle.set_k_d(yaml["k_d__roll"].as<float>()); // [constant]
};
