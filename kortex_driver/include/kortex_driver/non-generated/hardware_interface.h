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
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <kortex_driver/non-generated/kortex_arm_driver.h>
#include <kortex_driver/non-generated/kortex_command_interface.h>

// ros stuff
#include <ros/ros.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <ros/console.h>
#include "angles/angles.h"
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

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
  void read_loop(const double rate);
  void write_loop();

  void read();
  void write();
  void update();

  /**
   * Set the servoing mode. There are two servoing modes:
   * -
   * -
   */
  bool set_servoing_mode(const Kinova::Api::Base::ServoingMode& servoing_mode);

  /**
   * @brief Set the actuator low level control mode
   * @param mode
   * @return
   */
  bool set_actuators_control_mode(const KortexControlMode& mode);

  /**
   * @brief Send command to the hardware
   * @return
   */
  bool send_command();

  /**
   * @brief Set the hardware command variable
   */
  void set_hardware_command();


  /**
   * @brief Copy ros commands to hardware commands variable
   */
  void copy_commands();

  /**
   * @brief Dedicated function to switch between modes
   */
  void switch_mode();

  /**
   * @brief Set the joint limits from parameter file
   * @return false if unable to set the joint limits
   */
  bool set_joint_limits();

  /**
   * @brief Enforce limits and additional postprocessing
   * @return
   */
  void enforce_limits();

  /**
   * @brief Make sure the state is within the limits. If not, triggers the estop
   */
  void check_state();

  /**
   * @brief Publish commands as sensor_msgs/JointState in a realtime safe manner
   */
  void publish_commands();

  /**
   * @brief Publish state as sensor_msgs/JointState in a realtime safe manner
   */
  void publish_state();

  /**
   * @brief Returns the angle for a continuous joint when a_next is in the range [-PI, PI]
   * @param a_prev the previous continuous joint angle [-inf, inf]
   * @param a_next the next reading for the joint angle [-PI, PI]
   * @return the wrapped angle
   */
  double wrap_angle(const double a_prev, const double a_next) const;

 private:
  ros::Time last_time;
  controller_manager::ControllerManager* cm;
  hardware_interface::KortexCommandInterface jnt_cmd_interface;
  hardware_interface::JointStateInterface jnt_state_interface;

  std::vector<std::string> joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3",
                                          "arm_joint_4", "arm_joint_5", "arm_joint_6", "arm_joint_7"};
  int NDOF = joint_names.size();

  double pos[7];                         // [-PI, PI] converted readings from actuators
  double pos_cmd[7];                     // [-PI, PI] converted commands to actuators
  double pos_cmd_copy[7];                // intermediate variable to reduce locking time
  double pos_wrapped[7];                 // position reading with continuous revolution for continuous joints

  double vel[7];
  double vel_cmd[7];
  double vel_cmd_copy[7];

  double eff[7];
  double eff_cmd[7];
  double eff_cmd_copy[7];

  std::vector<joint_limits_interface::JointLimits> limits;

  hardware_interface::KortexControlMode mode;
  hardware_interface::KortexControlMode mode_copy;
  hardware_interface::KortexControlMode current_mode;

  // multi-threading
  std::atomic_bool stop_writing;    // when switching back to highlevel we need to stop the concurrent write
  std::mutex cmd_mutex;

  std::thread write_thread;
  std::thread read_update_thread;

  Feedback current_state;
  Kinova::Api::BaseCyclic::Command kortex_cmd;

  // publish state and command
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_command_pub_;

  ros::Time last_publish_time_;
  double publish_rate_;

  bool initialized_;
  ros::ServiceClient estop_client_;
};
}

std::ostream& operator<<(std::ostream& os, const joint_limits_interface::JointLimits& limits);


#endif