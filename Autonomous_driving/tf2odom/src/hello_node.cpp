/*
Custom hello world
Author: Fernando Espindola
*/

#include <ros/ros.h>
#include "hello.h"

/**
 * @brief main Declare a PointCatcher object and start interaction with the ROS environment
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char	**argv)	{

  // Initiate new ROS node named "catcher"
  ros::init(argc, argv,	"hello_people");

  // Create new PointCatcher object
  HelloTalker hello_talker;

  ros::spin();

  return 0;
}

