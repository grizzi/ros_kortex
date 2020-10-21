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

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "controller_manager/controller_manager.h"

#include <kortex_driver/non-generated/kortex_arm_driver.h>

// ros stuff
#include <ros/ros.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <ros/console.h>
#include "angles/angles.h"

using namespace Kinova::Api;
using namespace Kinova::Api::BaseCyclic;
using namespace Kinova::Api::Base;

namespace kortex_hardware_interface
{
class KortexHardwareInterface : hardware_interface::RobotHW, KortexArmDriver
{
 public:
  KortexHardwareInterface(ros::NodeHandle& nh);
  void read();
  void write();
  void update();
  ros::Time get_time();
  ros::Duration get_period();
  ~KortexHardwareInterface();

 private:
  void switch_mode(hardware_interface::JointCommandModes& new_mode);

 private:
  ros::Time last_time;
  controller_manager::ControllerManager* cm;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  hardware_interface::JointModeInterface jnt_mode_interface;

  BaseClient* m_base;
  BaseCyclicClient* m_basecyclic;

  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
  int NDOF = joint_names.size();
  double pos[7];
  double vel[7];
  double eff[7];
  double pos_cmd[7];
  double vel_cmd[7];
  double eff_cmd[7];
  double prev_pos_cmd[7];
  double prev_vel_cmd[7];
  double prev_eff_cmd[7];
  hardware_interface::JointCommandModes current_mode;
  hardware_interface::JointCommandModes mode[7];
};
}


#endif