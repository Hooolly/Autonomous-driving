#include <ros/ros.h>
#include "imu.h"

/*
 * @author Fernando Espindola
 */


ImuSimulator::ImuSimulator()
{

	ROS_INFO("-----------------------------");

	odom_subscriber = node.subscribe("/odom_stage", 1000, &ImuSimulator::sendIMU, this);

	imu_pub = node.advertise<sensor_msgs::Imu>("imu_data", 1000);
	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1000);

	seq = 0;

}



void ImuSimulator::sendIMU(const nav_msgs::Odometry msg){

	sensor_msgs::Imu imu_data;
	imu_data.header.seq = seq;
	imu_data.header.stamp = msg.header.stamp;
	imu_data.header.frame_id = "/base_link";

	tf::Quaternion orientation_quat;

  orientation_quat = tf::Quaternion(0,0,0,1);

  imu_data.orientation.x = orientation_quat[0]; //No orientation available
  imu_data.orientation.y = orientation_quat[1];
  imu_data.orientation.z = orientation_quat[2];
  imu_data.orientation.w = orientation_quat[3];

  imu_data.orientation_covariance[0] = -1.0;
  imu_data.orientation_covariance[4] = 10000000000000.0;
  imu_data.orientation_covariance[8] = 10000000000000.0;

  imu_data.angular_velocity.x = 0; //sensor outputs in rad/s
  imu_data.angular_velocity.y = 0;
  imu_data.angular_velocity.z = M_PI/8;
  imu_data.angular_velocity_covariance[0] = 0.0001;
  imu_data.angular_velocity_covariance[4] = 0.0001;
  imu_data.angular_velocity_covariance[8] = 0.0001;

  imu_data.linear_acceleration.x = 0; //my sensor output in g's
  imu_data.linear_acceleration.y = 0; 
  imu_data.linear_acceleration.z = 0; 
  imu_data.linear_acceleration_covariance[0] = 0.0001;
  imu_data.linear_acceleration_covariance[4] = 0.0001;
  imu_data.linear_acceleration_covariance[8] = 0.0001;


	nav_msgs::Odometry odom;

	odom.header = msg.header;
	odom.pose = msg.pose;
	odom.pose.covariance[0] = 0.00001;
	odom.pose.covariance[7] = 0.00001;
	odom.pose.covariance[14] = 1000000000000.0;	
	odom.pose.covariance[21] = 1000000000000.0;
	odom.pose.covariance[28] = 1000000000000.0;
	odom.pose.covariance[35] = 0.001;



	imu_pub.publish(imu_data);		
	odom_pub.publish(odom);

 		//tf::poseTFToMsg(, odom.pose.pose);

		seq++;

}


