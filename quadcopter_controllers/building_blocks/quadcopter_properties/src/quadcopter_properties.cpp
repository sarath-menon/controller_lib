#include "quadcopter_properties.h"

void QuadProperties::set_properties(std::string path) {
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
}
