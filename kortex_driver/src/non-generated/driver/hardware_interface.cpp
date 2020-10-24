/*!
 * @file     hardware_interface.cpp
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#include "kortex_driver/non-generated/hardware_interface.h"

using namespace hardware_interface;

KortexHardwareInterface::KortexHardwareInterface(ros::NodeHandle& nh) : KortexArmDriver(nh)
{

  for (std::size_t i = 0; i < NDOF; ++i)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    // connect and register the joint command interface
    hardware_interface::KortexCommandHandle cmd_handle(
        jnt_state_interface.getHandle(joint_names[i]), &pos_cmd[i], &vel_cmd[i], &eff_cmd[i], &mode);
    jnt_cmd_interface.registerHandle(cmd_handle);

  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_cmd_interface);

  for (size_t i = 0; i < 7; i++) {
    kortex_cmd.add_actuators();
  }
  set_command(true);
  current_mode = KortexControlMode::NO_MODE;
  set_actuators_control_mode(current_mode);

  last_time = ros::Time::now();
  cm = new controller_manager::ControllerManager(&*this);
}

KortexHardwareInterface::~KortexHardwareInterface(){
  set_actuators_control_mode(KortexControlMode::NO_MODE);
}

void KortexHardwareInterface::update_control()
{
  cm->update(this->get_time(), this->get_period());
}

void KortexHardwareInterface::read()
{
  Feedback current_state;
  current_state = m_base_cyclic->RefreshFeedback();
  for(int i = 0; i < current_state.actuators_size(); i++)
  {
    pos[i] = angles::normalize_angle(static_cast<double>(current_state.actuators(i).position()/180.0*M_PI));
    vel[i] = static_cast<double>(current_state.actuators(i).velocity()/180.0*M_PI);
    eff[i] = static_cast<double>(current_state.actuators(i).torque());
  }
}

void KortexHardwareInterface::write()
{
  if (mode != current_mode){
    ROS_INFO_STREAM("Switching to mode: " << mode);
    set_actuators_control_mode(mode);
  }

  // one mode for all joints
  if (current_mode == KortexControlMode::NO_MODE || current_mode == KortexControlMode::VELOCITY){
    return;
  }
  else if (current_mode == KortexControlMode::POSITION || current_mode == KortexControlMode::EFFORT){
    set_command();
    if (!send_command()){
      ROS_ERROR_STREAM_THROTTLE(1.0, "Failed to send commands.");
    }
  }
  else{
    ROS_WARN_STREAM_THROTTLE(1.0, "Unknown mode with id: " << mode);
  }
}


bool KortexHardwareInterface::set_servoing_mode(const Kinova::Api::Base::ServoingMode& mode) {
  bool success = false;
  Kinova::Api::Base::ServoingModeInformation servoing_mode;
  try
  {
    servoing_mode.set_servoing_mode(mode);
    m_base->SetServoingMode(servoing_mode);
    success = true;
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    ROS_ERROR_STREAM("Kortex exception: " << ex.what());
    ROS_ERROR_STREAM(
        "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(
            Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error& ex2)
  {
    ROS_ERROR_STREAM("runtime error: " << ex2.what());
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Unknown error.");
  }
  return success;
}

bool KortexHardwareInterface::set_actuators_control_mode(const KortexControlMode& mode) {
  Kinova::Api::ActuatorConfig::ControlModeInformation control_mode_info;
  try {
    if (mode == KortexControlMode::NO_MODE || mode == KortexControlMode::VELOCITY) {
      set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      current_mode = mode;
      return true;
    }
    else if (mode == KortexControlMode::POSITION) {
      set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);

    }
    else if (mode == KortexControlMode::EFFORT) {
      set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
      set_command(true);
      send_command();
    }

    for (size_t i = 1; i < 8; i++) {
      m_actuator_config->SetControlMode(control_mode_info, i);
    }
    current_mode = mode;
    return true;
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    ROS_ERROR_STREAM("Kortex exception: " << ex.what());
    ROS_ERROR_STREAM(
        "Error sub-code: " <<
        Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error& ex2)
  {
    ROS_ERROR_STREAM("runtime error: " << ex2.what());
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown error.");
  }
}

void KortexHardwareInterface::set_command(bool use_measured){
  kortex_cmd.set_frame_id(kortex_cmd.frame_id() + 1);  // unique-id to reject out-of-time frames
  if (kortex_cmd.frame_id() > 65535) {
    kortex_cmd.set_frame_id(0);
  }
  for (int idx = 0; idx < 7; idx++) {
    kortex_cmd.mutable_actuators(idx)->set_command_id(kortex_cmd.frame_id());

    if (use_measured){
      kortex_cmd.mutable_actuators(idx)->set_position(angles::normalize_angle_positive(pos[idx]*180.0/M_PI));
      kortex_cmd.mutable_actuators(idx)->set_torque_joint(eff[idx]);
    }
    else{
      kortex_cmd.mutable_actuators(idx)->set_position(angles::normalize_angle_positive(pos_cmd[idx]*180.0/M_PI));
      kortex_cmd.mutable_actuators(idx)->set_torque_joint(eff_cmd[idx]);
    }
  }
}

bool KortexHardwareInterface::send_command(){
  bool success = false;
  try{
    m_base_cyclic->Refresh(kortex_cmd, 0);
    success = true;
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    ROS_ERROR_STREAM("Kortex exception: " << ex.what());
    ROS_ERROR_STREAM(
        "Error sub-code: "
        << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error& ex2)
  {
    ROS_ERROR_STREAM("runtime error: " << ex2.what());
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Unknown error.");
  }
  return success;
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
