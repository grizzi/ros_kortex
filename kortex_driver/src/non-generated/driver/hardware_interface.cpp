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
  mode = current_mode;
  set_actuators_control_mode(current_mode);

  last_time = ros::Time::now();
  cm = new controller_manager::ControllerManager(&*this);
}

KortexHardwareInterface::~KortexHardwareInterface(){
  if (write_thread.joinable()) {
    write_thread.join();
  }

  set_actuators_control_mode(KortexControlMode::NO_MODE);

  if (read_update_thread.joinable())
    read_update_thread.join();

}

void KortexHardwareInterface::run(){
  write_thread = std::thread(&KortexHardwareInterface::write_loop, this);
  read_update_thread = std::thread(&KortexHardwareInterface::read_update_loop, this, 100);
  ROS_INFO_STREAM("Kinova ros controller interface is running.");
}

void KortexHardwareInterface::read_update_loop(const double f /* 1/sec */){
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  double elapsed_ms;
  double dt_ms = 1000./f;
  while (ros::ok()){
    start = std::chrono::steady_clock::now();
    read();
    update();
    copy_commands();
    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()/1e6;
    if (elapsed_ms > dt_ms)
      ROS_WARN_STREAM_THROTTLE(1.0, "Read and update took too long: " << elapsed_ms << " > " << dt_ms << " ms.");
    else{
      std:this_thread::sleep_for(std::chrono::nanoseconds((int)((dt_ms - elapsed_ms)*1e6)));
    }
  }
}

void KortexHardwareInterface::write_loop(){
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  double elapsed_ms;
  double dt_ms = 1.0;
  while (ros::ok()){
    start = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(cmd_mutex);
      write();
    }
    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()/1e6;
    if (elapsed_ms > dt_ms)
      ROS_WARN_STREAM_THROTTLE(1.0, "Write took too long: " << elapsed_ms << " > 1.0 ms.");
    else{
      std:this_thread::sleep_for(std::chrono::nanoseconds((int)((dt_ms-elapsed_ms)*1e6)));
    }
  }
}

void KortexHardwareInterface::read()
{
  current_state = m_base_cyclic->RefreshFeedback();
  for(int i = 0; i < current_state.actuators_size(); i++)
  {
    pos[i] = angles::normalize_angle(static_cast<double>(M_PI * current_state.actuators(i).position()/180.0));
    vel[i] = static_cast<double>(current_state.actuators(i).velocity()/180.0*M_PI);
    eff[i] = static_cast<double>(current_state.actuators(i).torque());
  }
}

void KortexHardwareInterface::update()
{
  cm->update(this->get_time(), this->get_period());

  if (mode != current_mode){
    ROS_INFO_STREAM("Switching to mode: " << mode);
    set_actuators_control_mode(mode);
  }
}

void KortexHardwareInterface::write()
{
  if (current_mode == KortexControlMode::NO_MODE || current_mode == KortexControlMode::VELOCITY)
  {
    return;
  }
  else if (current_mode == KortexControlMode::POSITION || current_mode == KortexControlMode::EFFORT)
  {
    set_command();
    if (!send_command()){
      ROS_ERROR_STREAM_THROTTLE(1.0, "Failed to send commands.");
    }
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Unknown mode: " << mode);
  }
}



bool KortexHardwareInterface::set_servoing_mode(const Kinova::Api::Base::ServoingMode& mode) {
  bool success = false;
  Kinova::Api::Base::ServoingModeInformation servoing_mode;
  try
  {
    servoing_mode.set_servoing_mode(mode);
    m_base->SetServoingMode(servoing_mode);
    ROS_INFO("New servoing mode set.");
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
      if (!set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING))
        return false;
      current_mode = mode;
      return true;
    }
    else if (mode == KortexControlMode::POSITION || mode == KortexControlMode::EFFORT) {
      if (!set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING)){
        return false;
      }
      set_command(true);
      send_command();
    }

    if (mode == KortexControlMode::POSITION){
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
    }
    else if (mode == KortexControlMode::EFFORT) {
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
    }

    for (size_t i = 1; i < 8; i++) {
      m_actuator_config->SetControlMode(control_mode_info, i);
    }
    ROS_INFO_STREAM("Mode switched to " << mode);
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

    if (use_measured) {
      for (int idx = 0; idx < 7; idx++) {
        kortex_cmd.mutable_actuators(idx)->set_command_id(kortex_cmd.frame_id());
        kortex_cmd.mutable_actuators(idx)->set_position(angles::normalize_angle_positive(pos[idx]) * 180.0 / M_PI);
        kortex_cmd.mutable_actuators(idx)->set_torque_joint(eff[idx]);
      }
    }
    else{
      for (int idx = 0; idx < 7; idx++) {
        kortex_cmd.mutable_actuators(idx)->set_command_id(kortex_cmd.frame_id());
        kortex_cmd.mutable_actuators(idx)->set_position(angles::normalize_angle_positive(pos_cmd_copy[idx])*180.0 / M_PI);
        kortex_cmd.mutable_actuators(idx)->set_torque_joint(eff_cmd_copy[idx]);
      }
    }
}

bool KortexHardwareInterface::send_command(){
  bool success = false;
  try{
    ROS_INFO_ONCE("Sending first low level command");
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

void KortexHardwareInterface::copy_commands(){
  std::lock_guard<std::mutex> lock(cmd_mutex);
  for (size_t i=0; i<7; i++){
    pos_cmd_copy[i] = (current_mode == KortexControlMode::EFFORT) ? pos[i] : pos_cmd[i]; // avoid following error
    vel_cmd_copy[i] = vel_cmd[i];
    eff_cmd_copy[i] = eff_cmd[i];
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
