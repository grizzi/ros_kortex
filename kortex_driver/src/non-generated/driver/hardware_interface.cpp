/*!
 * @file     hardware_interface.cpp
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#include "kortex_driver/non-generated/hardware_interface.h"

using namespace kortex_hardware_interface;

KortexHardwareInterface::KortexHardwareInterface(ros::NodeHandle& nh) : KortexArmDriver(nh)
{
  for (std::size_t i = 0; i < NDOF; ++i)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i]), &pos_cmd[i]);
    jnt_pos_interface.registerHandle(pos_handle);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_names[i]), &vel_cmd[i]);
    jnt_vel_interface.registerHandle(vel_handle);

    // connect and register the joint effort interface
    hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(joint_names[i]), &eff_cmd[i]);
    jnt_eff_interface.registerHandle(vel_handle);

    // connect and register the joint mode interface
    hardware_interface::JointModeHandle mode_handle(joint_names[i], &mode[i]);
    jnt_mode_interface.registerHandle(mode_handle);
  }

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);
  registerInterface(&jnt_vel_interface);
  registerInterface(&jnt_eff_interface);

  last_time = ros::Time::now();

  cm = new controller_manager::ControllerManager(&*this);

  // don't continue until ros control is up so we don't write stray commands
  ROS_DEBUG("Waiting for the controller spawner to be up...");
  // TODO set the correct service here
  ros::service::waitForService("/resources/manipulation/control/controller_spawner/get_loggers");
  ros::Duration(3.5).sleep(); // short sleep to allow controller to come up
  ROS_DEBUG("Found controller spawner.");

}

void KortexHardwareInterface::update()
{
  cm->update(this->get_time(), this->get_period());
}

void KortexHardwareInterface::read()
{
  Feedback current_state;
  current_state = m_basecyclic->RefreshFeedback();
  for(int i = 0; i < current_state.actuators_size(); i++)
  {
    pos[i] = angles::normalize_angle(static_cast<double>(current_state.actuators(i).position()/180.0*M_PI));
    vel[i] = static_cast<double>(current_state.actuators(i).velocity()/180.0*M_PI);
    eff[i] = static_cast<double>(current_state.actuators(i).torque());
  }
}

void KortexHardwareInterface::write()
{

  if (mode[0] != current_mode){
    switch_mode(mode[0]);
    current_mode = mode[0];
  }

  switch (current_mode){
    case hardware_interface::JointCommandModes::EMERGENCY_STOP:{
      break;
    }
    case hardware_interface::JointCommandModes::MODE_POSITION:{
      break;
    }
    case hardware_interface::JointCommandModes::MODE_VELOCITY:{
      break;
    }
    case hardware_interface::JointCommandModes::MODE_EFFORT:{
      break;
    }
    default:{
      break;
    }
  }
  /*double thr = 0.0;
  if((abs(cmd[0] - prev_cmd[0]) > thr)
      || (abs(cmd[1] - prev_cmd[1]) > thr)
      || (abs(cmd[2] - prev_cmd[2]) > thr)
      || (abs(cmd[3] - prev_cmd[3]) > thr)
      || (abs(cmd[4] - prev_cmd[4]) > thr)
      || (abs(cmd[5] - prev_cmd[5]) > thr)
      || (abs(cmd[6] - prev_cmd[6]) > thr)) // only write when commanded velocity != prev commanded velocity
  {
    auto action = Action();
    action.set_name("regular velocity write");
    action.set_application_data("");

    auto jointSpeeds = action.mutable_send_joint_speeds();

    for(size_t i = 0 ; i < NDOF; ++i)
    {
      auto jointSpeed = jointSpeeds->add_joint_speeds();
      jointSpeed->set_joint_identifier(i);
      jointSpeed->set_value(- cmd[i]*180.0/M_PI);
      jointSpeed->set_duration(0.0);
    }

    try
    {
      m_base->SendJointSpeedsCommand(*jointSpeeds);
    }
    catch (KDetailedException& ex)
    {
      ROS_WARN_THROTTLE(1, "Kortex exception");
      ROS_WARN_THROTTLE(1, "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
      ROS_WARN_THROTTLE(1, "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
      ROS_WARN_THROTTLE(1, "KINOVA exception description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex2)
    {
      ROS_INFO("Other Kortex exception");
    }

  }

  for(std::size_t ind = 0; ind < NDOF; ++ind)
  {
    prev_cmd[ind] = cmd[ind];
  }*/
}

void KortexHardwareInterface::switch_mode(hardware_interface::JointCommandModes &new_mode) {
  if (new_mode == current_mode) return;

  Kinova::Api::Base::ServoingModeInformation servoing_mode;
  switch (current_mode) {
    case hardware_interface::JointCommandModes::EMERGENCY_STOP: {
      break;
    }
    case hardware_interface::JointCommandModes::MODE_POSITION: {
      servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      m_base->SetServoingMode(servoing_mode);
      break;
    }
    case hardware_interface::JointCommandModes::MODE_VELOCITY: {
      servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      m_base->SetServoingMode(servoing_mode);
      break;
    }
    case hardware_interface::JointCommandModes::MODE_EFFORT: {
      servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      m_base->SetServoingMode(servoing_mode);
      break;
    }
    default: {
      ROS_WARN_STREAM("Unspecified mode: " << (int) new_mode);
    }
  }
}

ros::Time KortexHardwareInterface::get_time()
{
  return ros::Time::now();
}

ros::Duration KortexHardwareInterface::get_period()
{
  ros::Time current_time = ros::Time::now();
  ros::Duration period = current_time - last_time;
  last_time = current_time;
  return period;
}

KortexHardwareInterface::~KortexHardwareInterface() {}