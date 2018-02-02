/*
Custom pylon_detector
Author: Fernando Espindola
*/

#include <ros/ros.h>
#include "pylon_detector.h"

/**
 * @brief main Declare a PylonDetector object and start interaction with the ROS environment
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char	**argv)	{

  // Initiate new ROS node named "catcher"
  ros::init(argc, argv,	"pylon_detector");

  // Create new PointCatcher object
  PylonDetector pylon_detector;

  ros::spin();

  return 0;
}

