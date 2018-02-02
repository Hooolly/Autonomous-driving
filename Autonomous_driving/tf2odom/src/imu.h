#ifndef IMU_H
#define IMU_H

#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>



class ImuSimulator
{
	public:
		ImuSimulator();


	private:
		ros::NodeHandle node;
		ros::Subscriber odom_subscriber;

		ros::Publisher imu_pub;
		ros::Publisher odom_pub;


		tf::TransformListener listener;
		tf::StampedTransform transform;

		void sendIMU(const nav_msgs::Odometry msg);
		unsigned int seq; 



};

#endif // HELLO_H
