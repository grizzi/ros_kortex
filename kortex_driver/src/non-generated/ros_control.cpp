/*!
 * @file     ros_control.cpp
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#include "kortex_driver/non-generated/hardware_interface.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kortex_ros_control");

  // if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  //     ros::console::notifyLoggerLevelsChanged();
  // }

  ros::NodeHandle n;

  kortex_hardware_interface::KortexHardwareInterface kortex_ros_control(n);

  ros::spin();

  return 1;
}