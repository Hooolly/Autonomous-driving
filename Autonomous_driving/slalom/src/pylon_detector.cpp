#include <geometry_msgs/PointStamped.h>

#include "pylon_detector.h"

PylonDetector::PylonDetector() {

  // Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("scan", 1, &PylonDetector::scanCallback, this);

  // Advertise a publisher for the object found
  pylonPub = node.advertise<geometry_msgs::PointStamped>("/pylon",100);

  ROS_INFO("Initializing");

}


// Process the incoming laser scan message
void PylonDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{


  ROS_INFO("New LaserScan");
  // Find the closest range between the defined minimum and maximum angles
  int minExtIndex = ceil((MIN_EXT_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxExtIndex = floor((MAX_EXT_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

  float closestRange = scan->ranges[minIndex];
  float closestAngle = minIndex * scan->angle_increment - MIN_SCAN_ANGLE_RAD ;
  float closestAngleRad;

  float biggestRange = 0;
  float biggestAngle = 0;
  float biggestAngleRad;
  
  
  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
 
      if (scan->ranges[currIndex] < closestRange && scan->ranges[currIndex] > 0.002 ) {
          closestRange = scan->ranges[currIndex];
          closestAngleRad = currIndex * scan->angle_increment + scan->angle_min;
          closestAngle = closestAngleRad*180/M_PI;
        }

      if (scan->ranges[currIndex] > biggestRange && scan->ranges[currIndex] < 6 ) {
          biggestRange = scan->ranges[currIndex];
          biggestAngleRad = currIndex * scan->angle_increment + scan->angle_min;
          biggestAngle = biggestAngleRad*180/M_PI;
        }
        
        //Scan central zone
    }
    
 bool pylon_found = false;
 
 if( biggestRange - closestRange >= 0  ){

         ROS_INFO("In the first if");
	 pylon.header.stamp= scan->header.stamp; //ros::Time(0);
	 pylon.header.frame_id = scan->header.frame_id;
	 pylon.point.x = closestRange * cos(closestAngleRad);
	 pylon.point.y = closestRange * sin(closestAngleRad);
	 pylon.point.z = 0;

	 ROS_INFO_STREAM("Closest range: " << closestRange);
	 ROS_INFO("Object at x: %f y: %f ", pylon.point.x, pylon.point.y);

         bool pylon_found = true;
         pylonPub.publish(pylon);
	 
	}
		 
 /*if((closestRange <= 5.90) && pylon_found){
        //ROS_INFO("In the second if");
  	pylonPub.publish(pylon);
		
  }*/
  
}
