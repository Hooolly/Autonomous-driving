#include <geometry_msgs/Twist.h>

#include "stopper.h"

Stopper::Stopper() {
  keepMoving = true;
  sign = 1;

  // Advertise a new publisher for the simulated robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("base_scan", 1, &Stopper::scanCallback, this);
}

// Send a velocity command
void Stopper::moveForward() {
  geometry_msgs::Twist msg; // The default constructor will set all commands to 0
  msg.linear.x = FORWARD_SPEED_MPS;

 // Select angular speed proportional to the current deviation
  if(goalAngle > 5){
    msg.angular.z = (goalAngle/30) * M_PI/12;
    sign = 1; //Defines direction for the next angular move in case it's too close to wall
  }

  if(goalAngle < 5){
    msg.angular.z = -(goalAngle/30 * M_PI/12);
    sign = -1;
  }
 
  commandPub.publish(msg);
}


void Stopper::moveInCircle() {
  geometry_msgs::Twist msg; // The default constructor will set all commands to 0
  msg.linear.x = 0;//FORWARD_SPEED_MPS;
	msg.angular.z = M_PI/8;

/*
 // Select angular speed proportional to the current deviation
  if(goalAngle > 5){
    msg.angular.z = (goalAngle/30) * M_PI/12;
    sign = 1; //Defines direction for the next angular move in case it's too close to wall
  }

  if(goalAngle < 5){
    msg.angular.z = -(goalAngle/30 * M_PI/12);
    sign = -1;
  }
*/ 
  commandPub.publish(msg);
}


// Send an angular velocity 
void Stopper::moveAngular() {
  geometry_msgs::Twist msg; // The default constructor will set all commands to 0
  //msg.angular.z = M_PI/2;
  
 // if(goalAngle > 30){
  msg.angular.z = M_PI/2 * sign;
 /* } else if(goalAngle < -30){
  msg.angular.z = -10*M_PI/2;
  } else {
  msg.angular.z = -10*M_PI/2;  
  }*/
    
  commandPub.publish(msg);
}



// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Find the closest range between the defined minimum and maximum angles
  int minExtIndex = ceil((MIN_EXT_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxExtIndex = floor((MAX_EXT_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

  float closestRange = scan->ranges[minIndex];
  float closestAngle = minIndex * scan->angle_increment - MIN_SCAN_ANGLE_RAD ;

  float biggestRange = scan->ranges[minIndex];
  float biggestAngle = 0;
  
  
  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
      if (scan->ranges[currIndex] < closestRange) {
          closestRange = scan->ranges[currIndex];
          closestAngle = (currIndex * scan->angle_increment + scan->angle_min)*180/M_PI;
        }
      
      if (scan->ranges[currIndex] > biggestRange) {
          biggestRange = scan->ranges[currIndex];
          biggestAngle = (currIndex * scan->angle_increment + scan->angle_min)*180/M_PI;
        }
        
        //Scan central zone
    }
    
  for (int currIndex = minExtIndex + 1; currIndex <= minIndex; currIndex++) {
      
      if (scan->ranges[currIndex] > biggestRange) {
          biggestRange = scan->ranges[currIndex];
          biggestAngle = (currIndex * scan->angle_increment + scan->angle_min)*180/M_PI;
        }
        
        //Scan right zone
    }  
    
  for (int currIndex = maxIndex + 1; currIndex <= maxExtIndex; currIndex++) {
      
      if (scan->ranges[currIndex] > biggestRange) {
          biggestRange = scan->ranges[currIndex];
          biggestAngle = (currIndex * scan->angle_increment + scan->angle_min)*180/M_PI;
        }

       //Scan left zone 
        
    }
  
  goalAngle = biggestAngle;  

  ROS_INFO_STREAM("Closest range: " << closestRange);
  //ROS_INFO_STREAM("Closest angle: " << closestAngle);

  ROS_INFO_STREAM("Biggest range: " << biggestRange);
  //ROS_INFO_STREAM("Biggest angle: " << biggestAngle);
  //ROS_INFO_STREAM("Angle_increment: " << scan->angle_increment);

  if (closestRange < MIN_PROXIMITY_RANGE_M) {
      ROS_INFO("Stop!");
      keepMoving = false;
    }
  else if(biggestRange < 0.8 ){
  	  ROS_INFO("Stop! Rotate");
      keepMoving = false;
  }
  else{
    keepMoving = true;
    }
}

void Stopper::startMoving()
{
  ros::Rate rate(10);
  ROS_INFO("Start moving");

  // Keep spinning loop until user presses Ctrl+C
  while (ros::ok()) {
      // Continue moving if scan changes (we or the obstacle gets moved)
      /*if (keepMoving){
        moveForward();
        }
      else {
        moveAngular();      
      }*/
      //moveInCircle();
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep();
    }
}
