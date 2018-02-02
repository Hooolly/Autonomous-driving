#ifndef PYLON_DETECTOR_H
#define PYLON_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>

class PylonDetector {
  public:
    // Tunable parameters
    constexpr static double FORWARD_SPEED_MPS = 0.5;
    constexpr static double MIN_EXT_SCAN_ANGLE_RAD = -53.0/180*M_PI;
    constexpr static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
    constexpr static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
    constexpr static double MAX_EXT_SCAN_ANGLE_RAD = +53.0/180*M_PI;    
    constexpr static float MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max

    PylonDetector();


  private:
    ros::NodeHandle node;
    ros::Publisher pylonPub; // Publisher to the object found command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    geometry_msgs::PointStamped pylon;
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif // PYLON_DETECTOR_H
