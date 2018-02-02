#ifndef SLALOM_H
#define SLALOM_H

#include <math.h>

//ROS headers
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/Bool.h>
constexpr static double PI = 3.14159265358979323; // definition of PI value


class Slalom 
{

public:

	// Constructor
	Slalom();


  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  int setGoal(double x, double y, double w, double z);		//function for transmitting a goal position to move_base
  int setGoal(double x, double y, double yaw);
  int setGoal(double x, double y, double yaw, std::string frame);
  //void Getgoal();//function to generate waypoints

private:

  //declare Node
  ros::NodeHandle node;


  int result;
  double goal_x;
  double goal_y;
  double goal_yaw;

};


#endif // SLALOM_H
