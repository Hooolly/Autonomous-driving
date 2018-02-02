/*======================================
  This is the head file for slalom node
========================================*/
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




class Slalom {

	public:

		Slalom();
//********* in node main function ***************************************************
        // The initialization function, return a bool value
        bool init(ros::NodeHandle &nh);
        double test;
        double test1;
        double test2;
        double PI; // definitino of PI value

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        double angle_max;
        double angle_min;
        double angle_increment;
        double old_go1;
        //*********The update function, include callback function****************************
        void update(const ros::Time& time, const ros::Duration& period, ros::NodeHandle &nh);
        ros::Subscriber ScanInfo;
        sensor_msgs::LaserScan::ConstPtr scan;
        ros::NodeHandle nh;
        enum State {START, FIRST_P, SECOND_P, THIRD_P, FOURTH_P};
        State current_state;
        std::vector<double> old_position;
//*********** in function library ****************************************************
        double diff;//m
        int side_1 = 0;//index
        double max_width_pylon;//m
        int pylon_fram_index;
        double side_1d;
        double object_error;
        int num_pylon;
        std::vector<double> pylon_2D;
        std::vector<double> pylon_angle;//deg
        int goal_index;
        double goal_dist;
        double pylon_r;//m

        double Pylon_goal_x;
        double Pylon_goal_y;
        double Pylon_goal_yaw;
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
        void Slalom_Go();
        int  SendGoal(double x, double y, double yaw);
        void start();
        void first_p();
        void second_p();
        void third_p();
        void fourth_p();


	private:

        // Initial goal
        double InitGoal_x;
        double InitGoal_y;
        double InitGoal_yaw;
        // Step goal
        double Go_x;
        double Go_y;
        double Go_yaw;

        int result;
        int num;

};


#endif
