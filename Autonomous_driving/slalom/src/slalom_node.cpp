/*
Custom localization and object detection
Author: Fernando Espindola
*/

#include <ros/ros.h>
#include "slalom.h"

/**
 * @brief main Declare a PointCatcher object and start interaction with the ROS environment
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char	**argv)	{

  // Initiate new ROS node named "catcher"
  ros::init(argc, argv,	"slalom_node");

  // Create new PointCatcher object
  Slalom slalom;

  ros::spin();

  return 0;
}
