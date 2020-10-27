/*!
 * @file     ros_control.cpp
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#include <chrono>
#include "kortex_driver/non-generated/hardware_interface.h"

using namespace std::chrono;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kortex_ros_control");

  ros::NodeHandle n;
  hardware_interface::KortexHardwareInterface hw(n);

  hw.run();
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();                       // spin() will not return until the node has been shutdown
  //ros::spin();
  return 0;
}