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

  ros::NodeHandle n;
  hardware_interface::KortexHardwareInterface hw(n);

  while (ros::ok()){
    hw.read();
    hw.update_control();
    hw.write();
  }
  ros::spin();

  return 1;
}