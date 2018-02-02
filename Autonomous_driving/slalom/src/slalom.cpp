#include <ros/ros.h>
#include "slalom.h"
#include <math.h>

/*
 * @author Fernando Espindola
 */

Slalom::Slalom(){

	int sign = 1;

	for(int i = 1; i < 6; ++i) {
		double adj = - i * 0.01;
		double x =-0.3;
		double y = 0.34*sign; //(0.45 + adj )* sign;//0.34 without trailer * sign;//*(-1*i%2);
		double yaw =-5/180*PI*sign;

		/*double x = - 1.5 /2;
		double y = 0;
		double yaw =-45/180*PI*sign;*/

		std::string frame=std::string("pylon_")+std::to_string(i-1); 

		Slalom::setGoal(x, y, yaw, frame);

		//ROS_INFO("Moved to goal");

		sign = -sign;
	}

		//ROS_INFO("Finished");

};

int Slalom::setGoal(double x, double y, double yaw, std::string frame){


    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        //ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame;
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    
    ac.sendGoal(goal);
    //ROS_INFO("Sending goal");
    
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

    //ROS_INFO("Succeeded");
        return 1;
    }
    else {
    //ROS_INFO("Not Succeeded");
        return 0;
    }
}


int Slalom::setGoal(double x, double y, double yaw){


    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        //ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    
    ac.sendGoal(goal);
    //ROS_INFO("Sending goal");
    
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        return 1;
    }
    else {
        return 0;
    }
}
