/*======================================
  This is the main file for slalom node
========================================*/
#include <ros/ros.h>
#include "slalom_2.h"
#include <math.h>
#include <vector>
#include <iostream>


int main(int argc, char **argv){

    ros::init(argc, argv, "slalom_2_node");
    ros::NodeHandle nh;

    Slalom slalom; 

    ros::Rate loop_rate(1);
    slalom.init(nh);

    //****debug************
    //ROS_INFO("Got scan data1 in: scan %f", slalom.test);

    ros::Time init_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();


    while(ros::ok()){
        //****debug************
        // ROS_INFO("Current : %f", slalom.test );
        //*********************

        slalom.update(init_time, ros::Time::now() - last_time, nh);
        last_time = ros::Time::now();

        sleep(1);
        ros::spinOnce();

    }

    return 0;
}


