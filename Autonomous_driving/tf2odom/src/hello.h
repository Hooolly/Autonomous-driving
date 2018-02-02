#ifndef HELLO_H
#define HELLO_H

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



class HelloTalker 
{
	public:
		HelloTalker();


	private:
		ros::NodeHandle node;
		ros::Subscriber tf_subscriber;

		ros::Publisher odom_pub;

		tf::TransformListener listener;
		tf::StampedTransform transform;

		//void sendOdom(const tf2_msgs::TFMessage tfmsg);
		void sendOdom(const sensor_msgs::Imu tfmsg);
		unsigned int seq; 



};

#endif // HELLO_H
