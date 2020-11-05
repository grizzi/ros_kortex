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
  initialized_ = true;
  ROS_INFO_STREAM("Starting Kinova hardware interface in namespace: " << nh.getNamespace());
  for (std::size_t i = 0; i < NDOF; ++i)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos_wrapped[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    // connect and register the joint command interface
    hardware_interface::KortexCommandHandle cmd_handle(
        jnt_state_interface.getHandle(joint_names[i]), &pos_cmd[i], &vel_cmd[i], &eff_cmd[i], &mode);
    jnt_cmd_interface.registerHandle(cmd_handle);

  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_cmd_interface);

  bool limits_ok = set_joint_limits();
  if (!limits_ok){
    ROS_ERROR("Failed to set the joint limits");
    initialized_ = false;
  }

  // first read and fill command
  read();
  for (size_t i = 0; i < 7; i++) {
    pos_cmd[i] = pos[i];
    pos_wrapped[i] = pos[i];
    vel_cmd[i] = vel[i];
    eff_cmd[i] = eff[i];
    kortex_cmd.add_actuators();
  }

  realtime_state_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(m_node_handle, "/kinova_ros_control/joint_state", 4));
  realtime_command_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(m_node_handle, "/kinova_ros_control/joint_command", 4));

  for (unsigned i=0; i<7; i++){
    realtime_state_pub_->msg_.name.push_back(joint_names[i]);
    realtime_state_pub_->msg_.position.push_back(pos_wrapped[i]);
    realtime_state_pub_->msg_.velocity.push_back(vel[i]);
    realtime_state_pub_->msg_.effort.push_back(eff[i]);

    realtime_command_pub_->msg_.name.push_back(joint_names[i]);
    realtime_command_pub_->msg_.position.push_back(pos_cmd[i]);
    realtime_command_pub_->msg_.velocity.push_back(vel_cmd[i]);
    realtime_command_pub_->msg_.effort.push_back(eff_cmd[i]);
  }

  // at start up write to command the current state
  current_mode = KortexControlMode::NO_MODE;
  mode = current_mode;
  mode_copy = current_mode;
  set_actuators_control_mode(current_mode);

  last_time = ros::Time::now();
  cm = new controller_manager::ControllerManager(&*this);

  stop_writing = false;
  std::string emergency_stop_service_name = "/my_gen3/base/apply_emergency_stop";
  estop_client_ = m_node_handle.serviceClient<kortex_driver::ApplyEmergencyStop>(emergency_stop_service_name);
  if (!estop_client_.waitForExistence(ros::Duration(10.0))){
    ROS_ERROR_STREAM("Could not contact service: " << emergency_stop_service_name);
    initialized_ = false;
  }
}

KortexHardwareInterface::~KortexHardwareInterface(){
  if (write_thread.joinable()) {
    write_thread.join();
  }

  set_actuators_control_mode(KortexControlMode::NO_MODE);

  if (read_update_thread.joinable())
    read_update_thread.join();

}


bool KortexHardwareInterface::set_joint_limits(){

  bool limits_ok = true;
  limits.resize(joint_names.size());
  for (size_t i=0; i<joint_names.size(); i++){
    limits_ok &= joint_limits_interface::getJointLimits(joint_names[i], m_node_handle, limits[i]);
  }
  if (!limits_ok){
    return false;
  }
  std::stringstream joint_limits_string;
  for (size_t i=0; i<joint_names.size(); i++){
    joint_limits_string << "Limits " << joint_names[i] << ": " << limits[i] << std::endl;
  }
  ROS_INFO_STREAM(joint_limits_string.str());
  return true;
}

/// read loop functions
/// keep consistency with simulation: angle in range [-PI, PI] and unlimited continuous joints
void KortexHardwareInterface::read()
{
  current_state = m_base_cyclic->RefreshFeedback();
  for(int i = 0; i < current_state.actuators_size(); i++)
  {
    pos[i] = angles::normalize_angle(static_cast<double>(angles::from_degrees(current_state.actuators(i).position())));
    pos_cmd[i] = pos[i]; // avoid following errors (command position far from current position)

    // wrap angle for continuous joints (the even ones)
    pos_wrapped[i] = ((i % 2) == 0) ? wrap_angle(pos_wrapped[i], pos[i]) : pos[i];  

    vel[i] = static_cast<double>(angles::from_degrees(current_state.actuators(i).velocity()));
    eff[i] = static_cast<double>(-current_state.actuators(i).torque());
  }
}

void KortexHardwareInterface::update()
{
  cm->update(this->get_time(), this->get_period());
}

void KortexHardwareInterface::enforce_limits() {
  for (size_t i = 0; i < 7; i++) {
    pos_cmd[i] = std::max(std::min(pos_cmd[i], limits[i].max_position), limits[i].min_position);
    vel_cmd[i] = std::max(std::min(vel_cmd[i], limits[i].max_velocity), -limits[i].max_velocity);
    eff_cmd[i] = std::max(std::min(eff_cmd[i], limits[i].max_effort), -limits[i].max_effort);
  }
}

void KortexHardwareInterface::check_state() {
  bool ok = true;
  for (size_t i = 0; i < 7; i++) {
    ok &= limits[i].has_position_limits ? (pos_wrapped[i] < limits[i].max_position) && (pos_wrapped[i] > limits[i].min_position) : true;
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0, "Joint " << i << " violated position limits: " << pos_wrapped[i]);
      break;
    }

    ok &= limits[i].has_velocity_limits ? (vel[i] < limits[i].max_velocity) && (vel[i] > -limits[i].max_velocity) : true;
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0, "Joint " << i << " violated velocity limits: " << vel[i]);
      break;
    }

    ok &= limits[i].has_effort_limits ? (eff[i] < limits[i].max_effort) && (eff[i] > -limits[i].max_effort) : true;
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0, "Joint " << i << " violated effort limits: " << eff[i]);
      break;
    }
  }

  if (!ok){
    stop_writing = true;
    kortex_driver::ApplyEmergencyStopRequest req;
    kortex_driver::ApplyEmergencyStopResponse res;
    estop_client_.call(req, res);
  }

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

void KortexHardwareInterface::publish_state() {
  if (realtime_state_pub_->trylock()) {
    realtime_state_pub_->msg_.header.stamp = ros::Time::now();
    for (unsigned i = 0; i < 7; i++) {
      realtime_state_pub_->msg_.position[i] = pos_wrapped[i];
      realtime_state_pub_->msg_.velocity[i] = vel[i];
      realtime_state_pub_->msg_.effort[i] = eff[i];
    }
    realtime_state_pub_->unlockAndPublish();
  }
}

void KortexHardwareInterface::publish_commands() {
  if (realtime_command_pub_->trylock()) {
    realtime_command_pub_->msg_.header.stamp = ros::Time::now();
    for (unsigned i = 0; i < 7; i++) {
      realtime_command_pub_->msg_.position[i] = pos_cmd[i];
      realtime_command_pub_->msg_.velocity[i] = vel_cmd[i];
      realtime_command_pub_->msg_.effort[i] = eff_cmd[i];
    }
    realtime_command_pub_->unlockAndPublish();
  }
}

void KortexHardwareInterface::read_loop(const double f /* 1/sec */){
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  double elapsed_ms;
  double dt_ms = 1000./f;
  while (ros::ok()){
    start = std::chrono::steady_clock::now();

    // check the robot is safe
    check_state();

    // read the latest state
    read();

    // update the control commands
    update();

    // commands in the limits
    enforce_limits();

    // switch mode if required
    switch_mode();

    // copy ros commands to hardware commands
    copy_commands();

    // publish commands
    publish_commands();

    //publish state
    publish_state();

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
  if (!initialized_){
    ROS_ERROR_STREAM("Kinova hardware interface failed to initialize. Not running.");
    return;
  }

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
    kortex_cmd.mutable_actuators(idx)->set_position(angles::normalize_angle_positive(angles::to_degrees(pos_cmd_copy[idx])));
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

double KortexHardwareInterface::wrap_angle(const double a_prev, const double a_next) const{
  double a_wrapped;
  angles::shortest_angular_distance_with_large_limits(a_prev, a_next, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(), a_wrapped);
  a_wrapped = a_wrapped + a_prev;
  return a_wrapped;
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

std::ostream& operator<<(std::ostream& os, const joint_limits_interface::JointLimits& limits){
  os << "q_min: " << limits.min_position << "|| q_max: " << limits.max_position << "|| v_max: " << limits.max_velocity
  << "|| eff_max: " << limits.max_effort;
}