/*======================================
  This is the function library for slalom node
========================================*/
/*#   ____________________________________________
#   |                                          |
#   |  Technik Autonomer Systeme -TUM          |
#   |  Gruppe 03                               |
#   |                                          |
#   |  Author: WS 2017/2018-| Hao Xu |         |
#   |__________________________________________|


### INFO - Waypoints Slalom
###|           |########
###|   Kuchen! |========
###|
###|
###|
###|     C  3.2
###|           |========
###|    3.1    |########
###|           |##
###| 2.2 C     |##
###|           |##
###|    2.1    |##
###|           |##
###|     C 1.2 |##
###|           |##
###|    1.1    |##
###|           |##
###|  X  C     |##
###|           |##   1.1,1.2,2.1,...: Waypoint
###|   _____   |##   C: Cone
###|   | I |   |##   I: Initial position, Scan position
###|   |   |   |##   P: Preset Position
*/

#include "slalom.h"
#include <math.h>
Slalom::Slalom(){

};

// Callback function for laser scan data
void Slalom::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    scan = scan_in;
    test = (scan->angle_max -scan->angle_min)/scan->angle_increment;
    test1 = scan->ranges[0];
    test2 = scan->ranges[3];
    num = (scan->angle_max -scan->angle_min)/scan->angle_increment;

    ROS_INFO("num value : %f", num);

    angle_max = scan->angle_max;
    angle_min = scan->angle_min;
    angle_increment = scan->angle_increment;

    PI = 3.141592;
    diff = 0.6;//m
    side_1 = 0;//index
    num_pylon = 0;
    max_width_pylon = 0.2;//m
    pylon_r = 0.1;//m
    object_error = 0.04;//m

}

// Initial function, include subscriber
bool Slalom::init(ros::NodeHandle &nh){
    current_state = START;
    ScanInfo = nh.subscribe("scan", 10, &Slalom::ScanCallback, this);
 }

// Update function include subscriber
void Slalom::update(const ros::Time &time, const ros::Duration &period, ros::NodeHandle &nh){
    // store old laser scan data
    sensor_msgs::LaserScan::ConstPtr temp_laser = scan;
    ScanInfo = nh.subscribe("scan", 1000, &Slalom::ScanCallback, this);
    //ROS_INFO("Scan data frequenc: %f", temp_laser->scan_time);
    // check if the data has been updated
    if(Slalom::num != 0 ){
        ROS_INFO("Scan data succeed catch!");

        while(current_state != FOURTH_P){
            // Go to the state switch machine
            Slalom_Go();
        }
    }
    else{
        ROS_INFO("Error!!!!!!!!");
        ScanInfo = nh.subscribe("scan", 1000, &Slalom::ScanCallback, this);
    }

}

// State switch function
void Slalom::Slalom_Go(){

    ScanInfo = nh.subscribe("scan", 1000, &Slalom::ScanCallback, this);
    ros::spinOnce();
    switch(current_state){
        case START: start();
        break;
        case FIRST_P: first_p();
        break;
        case SECOND_P: second_p();
        break;
        case THIRD_P: third_p();
        case FOURTH_P: fourth_p();

    }
}

// Sending waypoints in "map frame" to the car
int Slalom::SendGoal(double x, double y, double yaw){
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for move_base action server");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    ac.sendGoal(goal);
    ROS_INFO("Sending goal %f, %f, %f:", x, y, yaw);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return 1;
    }
    else{
        return 0;
    }
}

// The start state
void Slalom::start(){
    InitGoal_x = 1.8; //m
    InitGoal_y = 0.6; //m
    InitGoal_yaw = -0.785;// deg

    ROS_INFO("start() state launched:)");

    result = SendGoal(InitGoal_x, InitGoal_y, 0);
    if (result == 1){
        ROS_INFO("Succeed: The car finish the first move:)");
        current_state = FIRST_P;

    }
    else{
        ROS_INFO("There is error in start state:(");
    }

}

// The first state
void Slalom::first_p(){

        double traffic_cone_distance;
        double traffic_cone_angle;
        double minDistance = 1.6 + 1.5;
        double maxDistance = 1.7 + 1.5;
        sensor_msgs::LaserScan::ConstPtr temp_scan = scan;

        //Now iterate over all sensor data points to find traffic cone
        for (int currIndex = 0; currIndex <= num; currIndex++)
        {
            //search for traffic cone position
            if ((temp_scan->ranges[currIndex] < maxDistance) && (temp_scan->ranges[currIndex] > minDistance))
            {

                traffic_cone_distance = temp_scan->ranges[currIndex];
                traffic_cone_angle = angle_min + angle_increment*currIndex;
            }
        }
        std::cout << "Distance to second traffic cone: " << traffic_cone_distance << std::endl;
        std::cout << "Angle to second traffic cone: " << traffic_cone_angle << std::endl;

        //Calculate next goal position
        Go_x = std::abs(std::cos(traffic_cone_angle))*(traffic_cone_distance);
        Go_y = -std::abs(std::sin(traffic_cone_angle))*(traffic_cone_distance);
        Go_yaw = PI/4; // 45 deg

        result = SendGoal(Go_x , Go_y/2 , Go_yaw);
            if(result == 1)
            {                
                ROS_INFO("Succeed: The car moves to second cone:)");
                SendGoal((Go_x-InitGoal_x) + Go_x, -0.6, 0);
                old_go1 = (Go_x-InitGoal_x) + Go_x;
                current_state = SECOND_P;
            }
            else
            {
                ROS_INFO("There has been an error in first state!");
        }

    }

//The second function
void Slalom::second_p(){

        double traffic_cone_distance;
        double traffic_cone_angle;

        int minIndex = ceil((angle_max / angle_increment) / 2);
        int maxIndex = floor((angle_max / angle_increment) / 2 - 90*((angle_max / angle_increment)/240));

        double minDistance = 1.6+3;
        double maxDistance = 1.7+3;

        sensor_msgs::LaserScan::ConstPtr temp_scan = scan;
        ROS_INFO("Scan data: %f", temp_scan->ranges[1]);
        // Iterate over all sensor data points to find traffic cone
         for (int currIndex = 0; currIndex <= num; currIndex++)
        {
            //search for traffic cone position
            if ((temp_scan->ranges[currIndex] < maxDistance) && (temp_scan->ranges[currIndex] > minDistance))
            {
                traffic_cone_distance = temp_scan->ranges[currIndex];
                traffic_cone_angle = angle_min + angle_increment*currIndex;
            }
        }
        std::cout << "Distance to third traffic cone: " << traffic_cone_distance << std::endl;
        std::cout << "Angle to third traffic cone: " << traffic_cone_angle << std::endl;

        //Calculate next goal position
        Go_x = std::abs(std::cos(traffic_cone_angle)*(traffic_cone_distance));
        Go_y = std::abs(std::sin(traffic_cone_angle)*traffic_cone_distance);
        Go_yaw = - PI / 4; // 45 deg

        result = SendGoal(Go_x-0.4 , Go_y/2 , Go_yaw);
        if(result == 1)
        {
            ROS_INFO("Succeed: The car moves to the third cone!");

            SendGoal((Go_x-old_go1) + Go_x, 0.6, 0);
            old_go1 = (Go_x-old_go1) + Go_x;
            current_state = THIRD_P;
        }
        else
        {
            ROS_INFO("There has been an error in second state!");
        }

    }

void Slalom::third_p(){
    double traffic_cone_distance;
    double traffic_cone_angle;
    int minIndex = ceil((angle_max / angle_increment) / 2);
    int maxIndex = floor((angle_max / angle_increment) / 2 - 90*((angle_max / angle_increment)/240));
    double minDistance = 1.55+ 4.5;
    double maxDistance = 1.7+ 4.5;

    sensor_msgs::LaserScan::ConstPtr temp_scan = scan;

    //Iterate over all sensor data points to find traffic cone
     for (int currIndex = 0; currIndex <= num; currIndex++)
    {
        //search for traffic cone position
        if ((temp_scan->ranges[currIndex] < maxDistance) && (temp_scan->ranges[currIndex] > minDistance))
        {
            traffic_cone_distance = temp_scan->ranges[currIndex];
            traffic_cone_angle = angle_min + angle_increment*currIndex;
        }
    }
    std::cout << "Distance to third traffic cone: " << traffic_cone_distance << std::endl;
    std::cout << "Angle to third traffic cone: " << traffic_cone_angle << std::endl;

    //Calculate next goal position
    Go_x = std::abs(std::cos(traffic_cone_angle)*traffic_cone_distance);
    Go_y = -std::abs(std::sin(traffic_cone_angle)*traffic_cone_distance);
    Go_yaw =  PI / 4; // 45 deg

     result = SendGoal(Go_x -0.9 , Go_y-0.6 , Go_yaw);
    if(result == 1)
    {
        ROS_INFO("Succeed: The car moves to fourth pole:)");
        //go to next state of state_machine

             SendGoal((Go_x-old_go1) + Go_x  , -0.6, 0);
             old_go1 = (Go_x-old_go1) + Go_x ;
             current_state = FOURTH_P;
    }
    else
    {
        ROS_INFO("There has been an error in third state!");
    }

}


void Slalom::fourth_p(){
    double traffic_cone_distance;
    double traffic_cone_angle;
    int minIndex = ceil((angle_max / angle_increment) / 2);
    int maxIndex = floor((angle_max / angle_increment) / 2 - 90*((angle_max / angle_increment)/240));
    double minDistance = 1.5+ 6;
    double maxDistance = 1.7+ 6;

    sensor_msgs::LaserScan::ConstPtr temp_scan = scan;

    //Now iterate over all sensor data points to find traffic cone
     for (int currIndex = 0; currIndex <= num; currIndex++)
    {
        //search for traffic cone position
        if ((temp_scan->ranges[currIndex] < maxDistance) && (temp_scan->ranges[currIndex] > minDistance))
        {

            traffic_cone_distance = temp_scan->ranges[currIndex];
            traffic_cone_angle = angle_min + angle_increment*currIndex;
        }
    }
    std::cout << "Distance to fourth traffic cone: " << traffic_cone_distance << std::endl;
    std::cout << "Angle to fourth traffic cone: " << traffic_cone_angle << std::endl;

}
