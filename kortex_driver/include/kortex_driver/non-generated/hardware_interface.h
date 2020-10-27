/*!
 * @file     hardware_interface.h
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#pragma once


#ifndef KORTEX_HARDWARE_INTERFACE_H
#define KORTEX_HARDWARE_INTERFACE_H

#include <cmath>
#include <memory>
#include <map>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "controller_manager/controller_manager.h"

#include <kortex_driver/non-generated/kortex_arm_driver.h>
#include <kortex_driver/non-generated/kortex_command_interface.h>

// ros stuff
#include <ros/ros.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <ros/console.h>
#include "angles/angles.h"

using namespace Kinova::Api;
using namespace Kinova::Api::BaseCyclic;
using namespace Kinova::Api::Base;

namespace hardware_interface
{

class KortexHardwareInterface : hardware_interface::RobotHW, KortexArmDriver
{
 public:
  KortexHardwareInterface() = delete;
  KortexHardwareInterface(ros::NodeHandle& nh);
  ~KortexHardwareInterface();
  void run();

  ros::Time get_time();
  ros::Duration get_period();

 private:
  void read_update_loop(const double rate);
  void write_loop();

  void read();
  void write();
  void update();

  bool set_servoing_mode(const Kinova::Api::Base::ServoingMode& servoing_mode);
  bool set_actuators_control_mode(const KortexControlMode& mode);
  bool send_command();
  void set_command(bool use_measured=false);
  void copy_commands();

 private:
  ros::Time last_time;
  controller_manager::ControllerManager* cm;
  hardware_interface::KortexCommandInterface jnt_cmd_interface;
  hardware_interface::JointStateInterface jnt_state_interface;

  std::vector<std::string> joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3",
                                          "arm_joint_4", "arm_joint_5", "arm_joint_6", "arm_joint_7"};
  int NDOF = joint_names.size();

  double pos[7];
  double vel[7];
  double eff[7];
  double pos_cmd[7];
  double vel_cmd[7];
  double eff_cmd[7];

  hardware_interface::KortexControlMode mode;
  hardware_interface::KortexControlMode current_mode;

  // multi-threading
  std::mutex cmd_mutex;
  double pos_cmd_copy[7];
  double vel_cmd_copy[7];
  double eff_cmd_copy[7];
  std::thread write_thread;
  std::thread read_update_thread;

  Feedback current_state;
  Kinova::Api::BaseCyclic::Command kortex_cmd;
};
}


#endif