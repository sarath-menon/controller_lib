#include "saturation_cutoff_mixer.h"
#include <iostream>

void QuadcopterMixer::set_mixer_matrix() {
  mixer_matrix_(0, 0) = 0.25;
  mixer_matrix_(0, 1) = 0.0;
  mixer_matrix_(0, 2) = 0.5 / arm_length;
  mixer_matrix_(0, 3) = 0.25 / k_t;

  mixer_matrix_(1, 0) = 0.25;
  mixer_matrix_(1, 1) = 0.5 / arm_length;
  mixer_matrix_(1, 2) = 0.0;
  mixer_matrix_(1, 3) = -0.25 / k_t;

  mixer_matrix_(2, 0) = 0.25;
  mixer_matrix_(2, 1) = 0.0;
  mixer_matrix_(2, 2) = -0.5 / arm_length;
  mixer_matrix_(2, 3) = 0.25 / k_t;

  mixer_matrix_(3, 0) = 0.25;
  mixer_matrix_(3, 1) = -0.5 / arm_length;
  mixer_matrix_(3, 2) = 0.0;
  mixer_matrix_(3, 3) = -0.25 / k_t;
}

cpp_msg::QuadMotorCommand &QuadcopterMixer::motor_mixer(
    const cpp_msg::ThrustTorqueCommand &thrust_torque_cmd) {
  matrix::Vector<float, 4> thrust_torque_cmd_;

  thrust_torque_cmd_(0) = thrust_torque_cmd.thrust;
  thrust_torque_cmd_(1) = thrust_torque_cmd.roll_torque;
  thrust_torque_cmd_(2) = thrust_torque_cmd.pitch_torque;
  thrust_torque_cmd_(3) = thrust_torque_cmd.yaw_torque;

  matrix::Vector<float, 4> motor_thrusts = mixer_matrix_ * thrust_torque_cmd_;

  for (int i = 0; i < 4; i++) {
    motor_thrusts(i) = math_helper::limit(motor_thrusts(i), motor_thrust_max,
                                          motor_thrust_min);
    motor_cmd.motorspeed[i] = sqrt(motor_thrusts(i) / k_f);
  }

  // std::cout << "Actual Motor 1 command " << motor_commands[0] <<
  // std::endl;

  //   std::cout << "Controller: f1:" << f1 << "\tf2:" << f2 << "\tf3:" <<
  //   f3
  //             << "\tf4:" << f4 << std::endl;
  // std::cout << "Net thrust and torque before  motor mixing:"
  //           << thrust_torque_cmd(0) << '\t' << thrust_torque_cmd(1)
  //           << std::endl;
  // std::cout << "Net thrust and torque after motor mixing:"
  //           << motor_thrusts(0) + motor_thrusts(1) + motor_thrusts(2) +
  //                  motor_thrusts(3)
  //           << '\t' << (motor_thrusts(1) - motor_thrusts(3)) * arm_length
  //           << std::endl;

  return motor_cmd;
}
