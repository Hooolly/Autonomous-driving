#include <ros/ros.h>
#include "hello.h"

/*
 * @author Fernando Espindola
 */


HelloTalker::HelloTalker()
{

	ROS_INFO("-----------------------------");

	//tf_subscriber = node.subscribe("/tf", 1000, &HelloTalker::sendOdom, this);

	tf_subscriber = node.subscribe("/imu/data", 100, &HelloTalker::sendOdom, this);
	
	odom_pub = node.advertise<nav_msgs::Odometry>("odom_alt", 1000);

	seq = 0;

}



//void HelloTalker::sendOdom(const tf2_msgs::TFMessage tfmsg){
void HelloTalker::sendOdom(const sensor_msgs::Imu tfmsg){
	
  try{

		ros::Time now  = ros::Time::now();
		//ros::Time past  = tfmsg.transforms[0].header.stamp;
		ros::Time past  = tfmsg.header.stamp;

		listener.waitForTransform("/odom_hector", "/base_link", past, ros::Duration(1.0));
		listener.lookupTransform("/odom_hector", "/base_link", past, transform);

		//ROS_INFO("Oi");

                nav_msgs::Odometry odom;

		//odom.header.stamp = transform.stamp_;
		//odom.header.stamp = tfmsg.transforms[0].header.stamp;
		odom.header.frame_id = "/odom_hector";
		geometry_msgs::PoseStamped pose_odom_in;
		geometry_msgs::PoseStamped pose_odom_out;

                pose_odom_in.header.frame_id = "/base_link";
                //pose_odom_in.header.stamp = tfmsg.transforms[0].header.stamp;
		pose_odom_in.header.stamp = past;

		geometry_msgs::Point origin;
		origin.x = 0;
		origin.y = 0;
		origin.z = 0;

		pose_odom_in.pose.position = origin;


		geometry_msgs::Quaternion q_out;
		tf::quaternionTFToMsg(tf::Quaternion(0,0,0,1),q_out);
		pose_odom_in.pose.orientation =q_out;

		//geometry_msgs::PoseWithCovariance pose_odom;

		listener.transformPose("/odom_hector", pose_odom_in, pose_odom_out);
		
		geometry_msgs::PoseWithCovariance pose_odom;		
	
		pose_odom.pose = pose_odom_out.pose;

		pose_odom.covariance[0] = 0.00001;
		pose_odom.covariance[7] = 0.00001;
		pose_odom.covariance[14] = 1000000000000.0;
		pose_odom.covariance[21] = 1000000000000.0;
		pose_odom.covariance[28] = 1000000000000.0;
		pose_odom.covariance[35] = 0.001;

		odom.pose = pose_odom;
		odom.header.seq = seq;
		odom.header.stamp = past;
		//odom.header.stamp = transform.stamp_;
                //odom.header.stamp = now;
		odom.header.frame_id = "/odom_hector";
 
		odom_pub.publish(odom);		

 		//tf::poseTFToMsg(, odom.pose.pose);

		seq++;

  }

	catch  (tf::TransformException ex) {

		ROS_WARN("/odom_hector to /base_link transform unavailable %s", ex.what());

	}


	

}


