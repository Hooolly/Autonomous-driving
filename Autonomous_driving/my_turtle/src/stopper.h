#ifndef STOPPER_H
#define STOPPER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Stopper {
  public:
    // Tunable parameters
    const static double FORWARD_SPEED_MPS = 0.5;
    const static double MIN_EXT_SCAN_ANGLE_RAD = -53.0/180*M_PI;
    const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
    const static double MAX_EXT_SCAN_ANGLE_RAD = +53.0/180*M_PI;    
    const static float MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max

    Stopper();
    void startMoving();

  private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    bool keepMoving; // Indicates whether the robot should continue moving
    float goalAngle; // Indicates whether the robot should continue moving
    signed char sign;

    void moveForward();
    void moveAngular();
    void moveInCircle();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif // STOPPER_H
