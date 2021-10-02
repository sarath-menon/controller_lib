#include "cascaded.h"

void Cascaded::set_timescales(std::string path) {
  // Load yaml file containing quadcopter properties
  YAML::Node timescale_yaml = YAML::LoadFile(path);

  // Load timescales from yaml file
  position_controller_rate = timescale_yaml["position_loop_rate"].as<float>();
  attitude_controller_rate = timescale_yaml["attitude_loop_rate"].as<float>();
  angular_velocity_controller_rate =
      timescale_yaml["angular_velocity_controller_rate"].as<float>();

  // Compute dt from timescale
  position_controller_dt = 1 / position_controller_rate;
  attitude_controller_dt = 1 / attitude_controller_rate;
  angular_velocity_controller_dt = 1 / angular_velocity_controller_rate;
}
