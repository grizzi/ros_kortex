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

  if (!nh.param("publish_rate", publish_rate_, 40.0)){
    ROS_WARN_STREAM("Publish rate not set. Publishing at " << publish_rate_ << " Hz.");
  }
  ROS_INFO_STREAM("Publish rate not set. Publishing at " << publish_rate_ << " Hz.");
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(m_node_handle, "/joint_commands", 4));

  for (unsigned i=0; i<7; i++){
    realtime_pub_->msg_.name.push_back(joint_names[i]);
    realtime_pub_->msg_.position.push_back(0.0);
    realtime_pub_->msg_.velocity.push_back(0.0);
    realtime_pub_->msg_.effort.push_back(0.0);
  }

  set_joint_limits();

  // first read and fill command
  read();
  for (size_t i = 0; i < 7; i++) {
    pos_cmd[i] = pos[i];
    vel_cmd[i] = vel[i];
    eff_cmd[i] = eff[i];
    kortex_cmd.add_actuators();
  }

  // at start up write to command the current state
  current_mode = KortexControlMode::NO_MODE;
  mode = current_mode;
  mode_copy = current_mode;
  set_actuators_control_mode(current_mode);

  last_time = ros::Time::now();
  cm = new controller_manager::ControllerManager(&*this);

  stop_writing = false;
}

KortexHardwareInterface::~KortexHardwareInterface(){
  if (write_thread.joinable()) {
    write_thread.join();
  }

  set_actuators_control_mode(KortexControlMode::NO_MODE);

  if (read_update_thread.joinable())
    read_update_thread.join();

}


void KortexHardwareInterface::set_joint_limits(){

  limits_ok = true;
  limits.resize(joint_names.size());
  for (size_t i=0; i<joint_names.size(); i++){
    limits_ok &= joint_limits_interface::getJointLimits(joint_names[i], m_node_handle, limits[i]);
  }

  if (!limits_ok) {
    ROS_ERROR("Failed to parse joint limits");
  }
  else{
    ROS_INFO("Limits parsed correctly");
  }
}

/// read loop functions

void KortexHardwareInterface::read()
{
  current_state = m_base_cyclic->RefreshFeedback();
  for(int i = 0; i < current_state.actuators_size(); i++)
  {
    pos[i] = angles::normalize_angle(static_cast<double>(M_PI * current_state.actuators(i).position()/180.0));
    vel[i] = static_cast<double>(M_PI * current_state.actuators(i).velocity()/180.0);
    eff[i] = static_cast<double>(-current_state.actuators(i).torque());
    pos_cmd[i] = pos[i]; // TODO(giuseppe) make sure this happens somewhere else to avoid following errors
  }
}

void KortexHardwareInterface::update()
{
  cm->update(this->get_time(), this->get_period());
}

bool KortexHardwareInterface::check_commands() {

  for (size_t i = 0; i < 7; i++) {
    pos_cmd[i] = (mode == KortexControlMode::EFFORT) ? pos[i] : pos_cmd[i]; // avoid following error

    if (limits_ok){
      //pos_cmd[i] = std::max(std::min(pos_cmd[i], limits[i].max_position), limits[i].min_position);
      //vel_cmd[i] = std::max(std::min(vel_cmd[i], limits[i].max_velocity), -limits[i].max_velocity);
      //eff_cmd[i] = std::max(std::min(eff_cmd[i], limits[i].max_effort), -limits[i].max_effort);
    }
  }
  return true;
}

void KortexHardwareInterface::switch_mode(){
  if (mode != current_mode){
    ROS_INFO_STREAM("Switching to mode: " << mode);
    set_actuators_control_mode(mode);
    return;
  }
}

void KortexHardwareInterface::copy_commands(){
  std::lock_guard<std::mutex> lock(cmd_mutex);
  mode_copy = current_mode;
  for (size_t i=0; i<7; i++){
    pos_cmd_copy[i] = pos_cmd[i]; // avoid following error
    vel_cmd_copy[i] = vel_cmd[i];
    eff_cmd_copy[i] = eff_cmd[i];
  }
}

void KortexHardwareInterface::publish_commands() {

  if (realtime_pub_->trylock()) {
    realtime_pub_->msg_.header.stamp = ros::Time::now();
    for (unsigned i = 0; i < 7; i++) {
      realtime_pub_->msg_.position[i] = pos_cmd[i];
      realtime_pub_->msg_.velocity[i] = vel_cmd[i];
      realtime_pub_->msg_.effort[i] = eff_cmd[i];
    }
    realtime_pub_->unlockAndPublish();

  }
}

void KortexHardwareInterface::read_loop(const double f /* 1/sec */){
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  double elapsed_ms;
  double dt_ms = 1000./f;
  while (ros::ok()){
    start = std::chrono::steady_clock::now();

    // read the latest state
    read();

    // update the control commands
    update();

    // check
    check_commands();

    // switch mode if required
    switch_mode();

    // copy ros commands to hardware commands
    copy_commands();

    // publish commands
    publish_commands();

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

    write();

    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()/1e6;
    if (elapsed_ms > dt_ms)
      ROS_WARN_STREAM_THROTTLE(1.0, "Write took too long: " << elapsed_ms << " > 1.0 ms.");
    else{
      std:this_thread::sleep_for(std::chrono::nanoseconds((int)((dt_ms-elapsed_ms)*1e6)));
    }
  }
}


void KortexHardwareInterface::write()
{
  if (stop_writing) return;

  std::lock_guard<std::mutex> lock(cmd_mutex);
  if (mode_copy == KortexControlMode::POSITION || mode_copy == KortexControlMode::EFFORT)
  {
    set_hardware_command();
    send_command();
  }
  else if (mode_copy == KortexControlMode::VELOCITY || mode_copy == KortexControlMode::NO_MODE){}
  else{
    ROS_WARN_STREAM("Unknown mode: " << mode_copy);
  }
}


/// Start main threads
void KortexHardwareInterface::run(){
  write_thread = std::thread(&KortexHardwareInterface::write_loop, this);
  read_update_thread = std::thread(&KortexHardwareInterface::read_loop, this, 100);
  ROS_INFO_STREAM("Kinova ros controller interface is running.");
}


/// Additional methods

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
    ROS_ERROR_STREAM_THROTTLE(5.0, "Kortex exception: " << ex.what());
    ROS_ERROR_STREAM_THROTTLE(5.0,
        "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(
            Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error& ex2)
  {
    ROS_ERROR_STREAM_THROTTLE(5.0, "runtime error: " << ex2.what());
  }
  catch (...)
  {
    ROS_ERROR_STREAM_THROTTLE(5.0, "Unknown error.");
  }
  return success;
}

bool KortexHardwareInterface::set_actuators_control_mode(const KortexControlMode& mode) {
  Kinova::Api::ActuatorConfig::ControlModeInformation control_mode_info;
  try {
    if (mode == KortexControlMode::NO_MODE || mode == KortexControlMode::VELOCITY) {
      stop_writing = true;
      if (!set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING))
        return false;
    }
    else if (mode == KortexControlMode::POSITION || mode == KortexControlMode::EFFORT) {
      stop_writing = false;
      if (!set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING)){
        return false;
      }
      set_hardware_command();
      send_command();
    }

    if (mode == KortexControlMode::EFFORT) {
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
    }
    else {
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
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

void KortexHardwareInterface::set_hardware_command(){
  kortex_cmd.set_frame_id(kortex_cmd.frame_id() + 1);  // unique-id to reject out-of-time frames
  if (kortex_cmd.frame_id() > 65535) {
    kortex_cmd.set_frame_id(0);
  }

  for (int idx = 0; idx < 7; idx++) {
    kortex_cmd.mutable_actuators(idx)->set_command_id(kortex_cmd.frame_id());
    kortex_cmd.mutable_actuators(idx)->set_position(angles::normalize_angle_positive(pos_cmd_copy[idx])*180.0 / M_PI);
    kortex_cmd.mutable_actuators(idx)->set_torque_joint(eff_cmd_copy[idx]);
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
