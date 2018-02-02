/*
Custom imu message
Author: Fernando Espindola
*/

#include <ros/ros.h>
#include "imu.h"

/**
 * @brief main Declare an ImuSimulator and start interaction with the ROS environment
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char	**argv)	{

  // Initiate new ROS node named "catcher"
  ros::init(argc, argv,	"imu_node");

  // Create new PointCatcher object
  ImuSimulator imu_sim;

  ros::spin();

  return 0;
}

