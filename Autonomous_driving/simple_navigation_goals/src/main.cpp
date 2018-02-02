/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

double offset = 2;
double offset1 = 1;

geometry_msgs::Pose waypoint1;
waypoint1.position.x = 3.95928692818;
waypoint1.position.y = 0.0;//0.0470529049635;
waypoint1.position.z = 0.0;
waypoint1.orientation.x = 0.0;
waypoint1.orientation.y = 0.0;
waypoint1.orientation.z = 0.0;//-0.00503681199705;
waypoint1.orientation.w = 1.0;//0.999987315182;
waypoints.push_back(waypoint1);
   
geometry_msgs::Pose waypoint2;
waypoint2.position.x = 9.38373661041;
waypoint2.position.y = 0;//0.0167733281851;
waypoint2.position.z = 0.0;
waypoint2.orientation.x = 0.0;
waypoint2.orientation.y = 0.0;
waypoint2.orientation.z = 0.0;//-0.0193467203687;
waypoint2.orientation.w = 1.0;//0.99981283469;
waypoints.push_back(waypoint2);
   
geometry_msgs::Pose waypoint3;
waypoint3.position.x = 15.914484024;
waypoint3.position.y = 0;//-0.179345741868;
waypoint3.position.z = 0.0;
waypoint3.orientation.x = 0.0;
waypoint3.orientation.y = 0.0;
waypoint3.orientation.z = 0.0;//-0.0187501017156;
waypoint3.orientation.w = 1.0;//0.99982420139;
waypoints.push_back(waypoint3);
   
geometry_msgs::Pose waypoint4;
waypoint4.position.x = 18.3643417358;
waypoint4.position.y = 0;//-0.091427706182;
waypoint4.position.z = 0.0;
waypoint4.orientation.x = 0.0;
waypoint4.orientation.y = 0.0;
waypoint4.orientation.z = 0.0;//-0.0324441313694;
waypoint4.orientation.w = 1.0;//0.999473550595;
waypoints.push_back(waypoint4);
   
geometry_msgs::Pose waypoint5;
waypoint5.position.x = 21.2629623413 + offset;
waypoint5.position.y = 0;//-0.199453905225;
waypoint5.position.z = 0.0;
waypoint5.orientation.x = 0.0;
waypoint5.orientation.y = 0.0;
waypoint5.orientation.z = 0.0484249085864;
waypoint5.orientation.w = 0.998826825946;
waypoints.push_back(waypoint5);
   
geometry_msgs::Pose waypoint6;
waypoint6.position.x = 22.5467853546 + offset;
waypoint6.position.y = 0.31575345993;
waypoint6.position.z = 0.0;
waypoint6.orientation.x = 0.0;
waypoint6.orientation.y = 0.0;
waypoint6.orientation.z = 0.337235740879;
waypoint6.orientation.w = 0.941420232985;
waypoints.push_back(waypoint6);
   
geometry_msgs::Pose waypoint7;
waypoint7.position.x = 22.8527183533 + offset;
waypoint7.position.y = 1.42244708538;
waypoint7.position.z = 0.0;
waypoint7.orientation.x = 0.0;
waypoint7.orientation.y = 0.0;
waypoint7.orientation.z = 0.739224531259;
waypoint7.orientation.w = 0.673459050266;
waypoints.push_back(waypoint7);
   
geometry_msgs::Pose waypoint8;
waypoint8.position.x = 22.8344039917 + offset + offset1;
waypoint8.position.y = 2.93205738068;
waypoint8.position.z = 0.0;
waypoint8.orientation.x = 0.0;
waypoint8.orientation.y = 0.0;
waypoint8.orientation.z = 0.677956159551;
waypoint8.orientation.w = 0.735102336907;
waypoints.push_back(waypoint8);
   
geometry_msgs::Pose waypoint9;
waypoint9.position.x = 22.3428077698 + offset + offset1;
waypoint9.position.y = 5.68161916733;
waypoint9.position.z = 0.0;
waypoint9.orientation.x = 0.0;
waypoint9.orientation.y = 0.0;
waypoint9.orientation.z = 0.704512286171;
waypoint9.orientation.w = 0.709691791297;
waypoints.push_back(waypoint9);
   
geometry_msgs::Pose waypoint10;
waypoint10.position.x = 22.4300460815 + offset + offset1;
waypoint10.position.y = 11.6387453079;
waypoint10.position.z = 0.0;
waypoint10.orientation.x = 0.0;
waypoint10.orientation.y = 0.0;
waypoint10.orientation.z = 0.713023870954;
waypoint10.orientation.w = 0.701139757431;
waypoints.push_back(waypoint10);
   
geometry_msgs::Pose waypoint11;
waypoint11.position.x = 22.6220359802 + offset + offset1;
waypoint11.position.y = 13.4501638412;
waypoint11.position.z = 0.0;
waypoint11.orientation.x = 0.0;
waypoint11.orientation.y = 0.0;
waypoint11.orientation.z = 0.711738674678;
waypoint11.orientation.w = 0.702444345815;
waypoints.push_back(waypoint11);
   
geometry_msgs::Pose waypoint12;
waypoint12.position.x = 22.5372505188 + offset ;
waypoint12.position.y = 14.3528146744;
waypoint12.position.z = 0.0;
waypoint12.orientation.x = 0.0;
waypoint12.orientation.y = 0.0;
waypoint12.orientation.z = 0.922113272235;
waypoint12.orientation.w = 0.386920034592;
waypoints.push_back(waypoint12);
   
geometry_msgs::Pose waypoint13;
waypoint13.position.x = 21.7100505829 + offset;
waypoint13.position.y = 14.7011537552;
waypoint13.position.z = 0.0;
waypoint13.orientation.x = 0.0;
waypoint13.orientation.y = 0.0;
waypoint13.orientation.z = 0.993077216368;
waypoint13.orientation.w = 0.11746336591;
waypoints.push_back(waypoint13);
   
geometry_msgs::Pose waypoint14;
waypoint14.position.x = 20.990398407 + offset;
waypoint14.position.y = 14.7158355713;
waypoint14.position.z = 0.0;
waypoint14.orientation.x = 0.0;
waypoint14.orientation.y = 0.0;
waypoint14.orientation.z = 0.999276554496;
waypoint14.orientation.w = 0.038031140333;
waypoints.push_back(waypoint14);
   
geometry_msgs::Pose waypoint15;
waypoint15.position.x = 19.759016037 + offset;
waypoint15.position.y = 14.6868495941;
waypoint15.position.z = 0.0;
waypoint15.orientation.x = 0.0;
waypoint15.orientation.y = 0.0;
waypoint15.orientation.z = 0.999874485337;
waypoint15.orientation.w = 0.0158434078399;
waypoints.push_back(waypoint15);
   
geometry_msgs::Pose waypoint16;
waypoint16.position.x = 17.9549732208 + offset;
waypoint16.position.y = 14.7264986038;
waypoint16.position.z = 0.0;
waypoint16.orientation.x = 0.0;
waypoint16.orientation.y = 0.0;
waypoint16.orientation.z = 0.999974978538;
waypoint16.orientation.w = 0.0070740580627;
waypoints.push_back(waypoint16);
   
geometry_msgs::Pose waypoint17;
waypoint17.position.x = 16.355134964 + offset;
waypoint17.position.y = 14.7430343628;
waypoint17.position.z = 0.0;
waypoint17.orientation.x = 0.0;
waypoint17.orientation.y = 0.0;
waypoint17.orientation.z = 0.999995957075;
waypoint17.orientation.w = -0.0028435601608;
waypoints.push_back(waypoint17);



/* Start of acml waypoints
geometry_msgs::Pose waypoint1  ;
waypoint1.position.x = 20.0704479218 ;
waypoint1.position.y = 4.06958198547 ;
waypoint1.position.z = 0 ;
waypoint1.orientation.x = 0 ;
waypoint1.orientation.y = 0 ;
waypoint1.orientation.z = -0.9949 ;
waypoint1.orientation.w = 0.1003 ;
waypoints.push_back(waypoint1)   ;
   
geometry_msgs::Pose waypoint2  ;
waypoint2.position.x = 13.0902881622 ;
waypoint2.position.y = 2.9260392189 ;
waypoint2.position.z = 0.0 ;
waypoint2.orientation.x = 0.0 ;
waypoint2.orientation.y = 0.0 ;
waypoint2.orientation.z = 0.996659118982 ;
waypoint2.orientation.w = -0.0816737445551 ;
waypoints.push_back(waypoint2)   ;
   
geometry_msgs::Pose waypoint3  ;
waypoint3.position.x = 9.04099941254 ;
waypoint3.position.y = 2.15270590782 ;
waypoint3.position.z = 0.0 ;
waypoint3.orientation.x = 0.0 ;
waypoint3.orientation.y = 0.0 ;
waypoint3.orientation.z = 0.99405296055 ;
waypoint3.orientation.w = -0.108897711742 ;
waypoints.push_back(waypoint3)   ;
   
geometry_msgs::Pose waypoint4  ;
waypoint4.position.x = 4.4561457634 ;
waypoint4.position.y = 1.31807935238 ;
waypoint4.position.z = 0.0 ;
waypoint4.orientation.x = 0.0 ;
waypoint4.orientation.y = 0.0 ;
waypoint4.orientation.z = 0.995768988439 ;
waypoint4.orientation.w = -0.0918919020519 ;
waypoints.push_back(waypoint4)   ;
   
geometry_msgs::Pose waypoint5  ;
waypoint5.position.x = -0.871821224689 ;
waypoint5.position.y = 0.525436997414 ;
waypoint5.position.z = 0.0 ;
waypoint5.orientation.x = 0.0 ;
waypoint5.orientation.y = 0.0 ;
waypoint5.orientation.z = 0.998562503299 ;
waypoint5.orientation.w = -0.0535996922079 ;
waypoints.push_back(waypoint5)   ;
   
geometry_msgs::Pose waypoint6  ;
waypoint6.position.x = -2.34505558014 ;
waypoint6.position.y = 0.155026063323 ;
waypoint6.position.z = 0.0 ;
waypoint6.orientation.x = 0.0 ;
waypoint6.orientation.y = 0.0 ;
waypoint6.orientation.z = 0.996088988094 ;
waypoint6.orientation.w = -0.0883556891033 ;
waypoints.push_back(waypoint6)   ;
   
geometry_msgs::Pose waypoint7  ;
waypoint7.position.x = -3.40825986862 ;
waypoint7.position.y = -0.189544320107 ;
waypoint7.position.z = 0.0 ;
waypoint7.orientation.x = 0.0 ;
waypoint7.orientation.y = 0.0 ;
waypoint7.orientation.z = 0.873132682032 ;
waypoint7.orientation.w = -0.487482635144 ;
waypoints.push_back(waypoint7)   ;
   
geometry_msgs::Pose waypoint8  ;
waypoint8.position.x = -4.01750612259 ;
waypoint8.position.y = -1.1074719429 ;
waypoint8.position.z = 0.0 ;
waypoint8.orientation.x = 0.0 ;
waypoint8.orientation.y = 0.0 ;
waypoint8.orientation.z = -0.673403549442 ;
waypoint8.orientation.w = 0.739275090611 ;
waypoints.push_back(waypoint8)   ;
   
geometry_msgs::Pose waypoint9  ;
waypoint9.position.x = -3.78013730049 ;
waypoint9.position.y = -3.23311185837 ;
waypoint9.position.z = 0.0 ;
waypoint9.orientation.x = 0.0 ;
waypoint9.orientation.y = 0.0 ;
waypoint9.orientation.z = -0.66598678938 ;
waypoint9.orientation.w = 0.745963535551 ;
waypoints.push_back(waypoint9)   ;
   
geometry_msgs::Pose waypoint10  ;
waypoint10.position.x = -3.10026717186 ;
waypoint10.position.y = -6.48121786118 ;
waypoint10.position.z = 0.0 ;
waypoint10.orientation.x = 0.0 ;
waypoint10.orientation.y = 0.0 ;
waypoint10.orientation.z = -0.664162463797 ;
waypoint10.orientation.w = 0.747588270161 ;
waypoints.push_back(waypoint10)   ;
   
geometry_msgs::Pose waypoint11  ;
waypoint11.position.x = -3.10241127014 ;
waypoint11.position.y = -9.37051963806 ;
waypoint11.position.z = 0.0 ;
waypoint11.orientation.x = 0.0 ;
waypoint11.orientation.y = 0.0 ;
waypoint11.orientation.z = -0.657676622169 ;
waypoint11.orientation.w = 0.753300378769 ;
waypoints.push_back(waypoint11)   ;
   
geometry_msgs::Pose waypoint12  ;
waypoint12.position.x = -2.40107417107 ;
waypoint12.position.y = -11.4375839233 ;
waypoint12.position.z = 0.0 ;
waypoint12.orientation.x = 0.0 ;
waypoint12.orientation.y = 0.0 ;
waypoint12.orientation.z = -0.675152803349 ;
waypoint12.orientation.w = 0.73767790541 ;
waypoints.push_back(waypoint12)   ;
   
geometry_msgs::Pose waypoint13  ;
waypoint13.position.x = -2.27468037605 ;
waypoint13.position.y = -13.5717010498 ;
waypoint13.position.z = 0.0 ;
waypoint13.orientation.x = 0.0 ;
waypoint13.orientation.y = 0.0 ;
waypoint13.orientation.z = -0.685579388895 ;
waypoint13.orientation.w = 0.727997871922 ;
waypoints.push_back(waypoint13)   ;
   
geometry_msgs::Pose waypoint14  ;
waypoint14.position.x = -2.14665317535 ;
waypoint14.position.y = -14.4794988632 ;
waypoint14.position.z = 0.0 ;
waypoint14.orientation.x = 0.0 ;
waypoint14.orientation.y = 0.0 ;
waypoint14.orientation.z = -0.180637066306 ;
waypoint14.orientation.w = 0.983549820943 ;
waypoints.push_back(waypoint14)   ;
   
geometry_msgs::Pose waypoint15  ;
waypoint15.position.x = -1.19431567192 ;
waypoint15.position.y = -14.5927934647 ;
waypoint15.position.z = 0.0 ;
waypoint15.orientation.x = 0.0 ;
waypoint15.orientation.y = 0.0 ;
waypoint15.orientation.z = 0.0756546830712 ;
waypoint15.orientation.w = 0.997134077709 ;
waypoints.push_back(waypoint15)   ;
   
geometry_msgs::Pose waypoint16  ;
waypoint16.position.x = 0.558244407177 ;
waypoint16.position.y = -14.2049789429 ;
waypoint16.position.z = 0.0 ;
waypoint16.orientation.x = 0.0 ;
waypoint16.orientation.y = 0.0 ;
waypoint16.orientation.z = 0.0837184546864 ;
waypoint16.orientation.w = 0.996489448185 ;
waypoints.push_back(waypoint16)   ;
   
geometry_msgs::Pose waypoint17  ;
waypoint17.position.x = 3.36849999428 ;
waypoint17.position.y = -13.6421480179 ;
waypoint17.position.z = 0.0 ;
waypoint17.orientation.x = 0.0 ;
waypoint17.orientation.y = 0.0 ;
waypoint17.orientation.z = 0.0163556269802 ;
waypoint17.orientation.w = 0.999866237787 ;
waypoints.push_back(waypoint17)   ;
   
geometry_msgs::Pose waypoint18  ;
waypoint18.position.x = 5.26797580719 ;
waypoint18.position.y = -13.3185577393 ;
waypoint18.position.z = 0.0 ;
waypoint18.orientation.x = 0.0 ;
waypoint18.orientation.y = 0.0 ;
waypoint18.orientation.z = 0.0181353953338 ;
waypoint18.orientation.w = 0.999835540195 ;
waypoints.push_back(waypoint18)   ;

End of acml waypoints
*/

/*


    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 15.000;
    waypoint1.position.y = 3.000;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0;
    waypoint1.orientation.y = 0;
    waypoint1.orientation.z = -0.9949;
    waypoint1.orientation.w = 0.1003;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 9.000;
    waypoint2.position.y = 2.000;
    waypoint2.position.z = 0.000;
    waypoint2.orientation.x = 0;
    waypoint2.orientation.y = 0;
    waypoint2.orientation.z = -0.9931;
    waypoint2.orientation.w = 0.1168;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 0.4906;
    waypoint3.position.y = 0.6799;
    waypoint3.position.z = 0.000;
    waypoint3.orientation.x = 0;
    waypoint3.orientation.y = 0;
    waypoint3.orientation.z = -0.9946;
    waypoint3.orientation.w = 0.1033;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = -3.800;
    waypoint4.position.y = -1.000;
    waypoint4.position.z = 0.000;
    waypoint4.orientation.x = 0;
    waypoint4.orientation.y = 0;
    waypoint4.orientation.z = -0.6608;
    waypoint4.orientation.w = 0.7505;
    waypoints.push_back(waypoint4);

    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = -3.100;
    waypoint5.position.y = -5.300;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0;
    waypoint5.orientation.y = 0;
    waypoint5.orientation.z = -0.6620;
    waypoint5.orientation.w = 0.7345;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = -3.000;
    waypoint6.position.y = -8.000;
    waypoint6.position.z = 0.000;
    waypoint6.orientation.x = 0;
    waypoint6.orientation.y = 0;
    waypoint6.orientation.z = -0.6786;
    waypoint6.orientation.w = 0.7345;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;
    waypoint7.position.x = -4.000;
    waypoint7.position.y = -12.300;
    waypoint7.position.z = 0.000;
    waypoint7.orientation.x = 0;
    waypoint7.orientation.y = 0;
    waypoint7.orientation.z = -0.9950;
    waypoint7.orientation.w = 0.0998;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;
    waypoint8.position.x = -7.500;
    waypoint8.position.y = -11.000;
    waypoint8.position.z = 0.000;
    waypoint8.orientation.x = 0;
    waypoint8.orientation.y = 0;
    waypoint8.orientation.z = 0.8011;
    waypoint8.orientation.w = 0.5985;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;
    waypoint9.position.x = -7.500;
    waypoint9.position.y = -7.500;
    waypoint9.position.z = 0.000;
    waypoint9.orientation.x = 0;
    waypoint9.orientation.y = 0;
    waypoint9.orientation.z = -0.6663;
    waypoint9.orientation.w = 0.7457;
    waypoints.push_back(waypoint9);

    geometry_msgs::Pose waypoint10;
    waypoint10.position.x = -7.400;
    waypoint10.position.y = -5.700;
    waypoint10.position.z = 0.000;
    waypoint10.orientation.x = 0;
    waypoint10.orientation.y = 0;
    waypoint10.orientation.z = -0.6282;
    waypoint10.orientation.w = 0.7781;
    waypoints.push_back(waypoint10);

    geometry_msgs::Pose waypoint11;
    waypoint11.position.x = -5.500;
    waypoint11.position.y = -3.100;
    waypoint11.position.z = 0.000;
    waypoint11.orientation.x = 0;
    waypoint11.orientation.y = 0;
    waypoint11.orientation.z = 0.0292;
    waypoint11.orientation.w = 0.9996;
    waypoints.push_back(waypoint11);

    geometry_msgs::Pose waypoint12;
    waypoint12.position.x = -3.000;
    waypoint12.position.y = -5.700;
    waypoint12.position.z = 0.000;
    waypoint12.orientation.x = 0;
    waypoint12.orientation.y = 0;
    waypoint12.orientation.z = -0.6786;
    waypoint12.orientation.w = 0.7345;
    waypoints.push_back(waypoint12);

    geometry_msgs::Pose waypoint13;
    waypoint13.position.x = -3.000;
    waypoint13.position.y = -12.000;
    waypoint13.position.z = 0.000;
    waypoint13.orientation.x = 0;
    waypoint13.orientation.y = 0;
    waypoint13.orientation.z = -0.6216;
    waypoint13.orientation.w = 0.7833;
    waypoints.push_back(waypoint13);

    geometry_msgs::Pose waypoint14;
    waypoint14.position.x = -1.120;
    waypoint14.position.y = -14.600;
    waypoint14.position.z = 0.000;
    waypoint14.orientation.x = 0;
    waypoint14.orientation.y = 0;
    waypoint14.orientation.z = 0.0292;
    waypoint14.orientation.w = 0.9996;
    waypoints.push_back(waypoint14);

    geometry_msgs::Pose waypoint15;
    waypoint15.position.x = -1.100;
    waypoint15.position.y = -14.300;
    waypoint15.position.z = 0.000;
    waypoint15.orientation.x = 0;
    waypoint15.orientation.y = 0;
    waypoint15.orientation.z = 0.0791;
    waypoint15.orientation.w = 0.9969;
    waypoints.push_back(waypoint15);

    geometry_msgs::Pose waypoint16;
    waypoint16.position.x = 5.000;
    waypoint16.position.y = -13.600;
    waypoint16.position.z = 0.000;
    waypoint16.orientation.x = 0;
    waypoint16.orientation.y = 0;
    waypoint16.orientation.z = 0.0791;
    waypoint16.orientation.w = 0.9969;
    waypoints.push_back(waypoint16);

*/


    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }
    return 0;
}
