/*!
 * @file     kortex_command_interface.h
 * @author   Giuseppe Rizzi
 * @date     24.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <array>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface {

/**
 * Handle to read and command the arm with different modes
 */
enum struct KortexControlMode{
  NO_MODE = -1,
  POSITION  = 0,
  VELOCITY = 1,
  EFFORT = 2
};


class KortexCommandHandle : public hardware_interface::JointStateHandle {
 public:
  using mode_t = KortexControlMode;

  KortexCommandHandle() = delete;

  /**
   * Creates an instance of a KortexCommandHandle.
   *
   * @param[in] joint_state_handle state handle.
   * @param[in] command A reference to the joint command wrapped by this handle.
   * @param[in] mode A reference to the command mode wrapped by this handle.
   */
  KortexCommandHandle(const JointStateHandle& joint_state_handle, double* pos, double* vel, double* eff, mode_t* mode)
      : JointStateHandle(joint_state_handle), pos_(pos), vel_(vel), eff_(eff), mode_(mode) {}

  /**
   * Set the current mode
   *
   * @param[in] command Command to set.
   */
  void setMode(const mode_t& mode) noexcept { *mode_ = mode; }

  /**
   * Sets the given command for the current mode.
   *
   * @param[in] command Command to set.
   */
  void setCommand(double& cmd){
    if (*mode_ == KortexControlMode::EFFORT) *eff_ = cmd;
    else if (*mode_ == KortexControlMode::EFFORT) *pos_ = cmd;
    else if (*mode_ == KortexControlMode::VELOCITY) *vel_ = cmd;
  }

  /**
   * Gets the given command for the current mode.
   *
   * @param[in] command Command to set.
   */
  double getCommand(){
    if (*mode_ == KortexControlMode::EFFORT) return *eff_;
    else if (*mode_ == KortexControlMode::EFFORT) return *pos_;
    else if (*mode_ == KortexControlMode::VELOCITY) return *vel_;
  }

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   * @param[in] elbow Elbow to set.
   */
  void setPositionCommand(double& cmd) noexcept { *pos_ = cmd; }
  void setVelocityCommand(double& cmd) noexcept { *vel_ = cmd; }
  void setEffortCommand(double& cmd) noexcept { *eff_ = cmd; }


  /**
   * Gets the current command.
   *
   * @return Current command.
   */
  const double& getPositionCommand() const noexcept { return *pos_; }
  const double& getVelocityCommand() const noexcept { return *vel_; }
  const double& getEffortCommand() const noexcept { return *eff_; }
  const mode_t& getMode() const noexcept { return *mode_; }


 private:
  double* pos_;
  double* vel_;
  double* eff_;
  mode_t* mode_;
};

/**
 * Hardware interface to command joints
 */
class KortexCommandInterface
    : public hardware_interface::HardwareResourceManager<KortexCommandHandle, hardware_interface::ClaimResources> {};


}  // namespace hardware_interface

std::ostream& operator<<(std::ostream& os, const hardware_interface::KortexControlMode& mode){
  if ( mode == hardware_interface::KortexControlMode::NO_MODE )
    return os<< " NO_MODE";
  else if ( mode == hardware_interface::KortexControlMode::POSITION )
    return os<< " POSITION";
  else if ( mode == hardware_interface::KortexControlMode::VELOCITY )
    return os<< " VELOCITY";
  else if ( mode == hardware_interface::KortexControlMode::EFFORT )
    return os<< " EFFORT";
  else
    return os<< " UNKNOWN (" << (int)mode << ")" ;
}
