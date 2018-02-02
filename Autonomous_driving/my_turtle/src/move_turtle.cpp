#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

using namespace std;

double pos_x = 0;

// Topic messages callback
void poseCallback(const turtlesim::PoseConstPtr& msg) {
//    ROS_INFO_STREAM("x: " << msg->x << ", y: " << msg->y);
    pos_x = msg->x;
}

int main(int argc, char **argv) {
    const double FORWARD_SPEED_MPS = 2;

    // Initialize the node
    ros::init(argc, argv, "move_turtle");
    ros::NodeHandle node;

    string robot_name = argc > 1 ? argv[1] : "turtle1";

    ROS_INFO_STREAM("Considering " << robot_name << " as our robot.");

    // A publisher for the movement data
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);

    // A listener for pose
    ros::Subscriber sub = node.subscribe(robot_name + "/pose", 10, poseCallback);


    // Drive forward at a given speed. The robot points up the x-axis.
    // The default constructor will set all commands to 0
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED_MPS;

    // Loop at 10Hz, publishing movement commands until we shut down
    ros::Rate rate(1);
    ROS_INFO("Starting to move forward");
    int i=0, wait_counter=0;
    while (ros::ok()) {

	if (wait_counter >= 4){

		msg.linear.x = 2;
		msg.angular.z = 1.570796327;
		pub.publish(msg);
		wait_counter = 0;

	}else {
            ++wait_counter;
          }
        // Allow processing of incoming messages
        ros::spinOnce();
        rate.sleep();
    }
}
