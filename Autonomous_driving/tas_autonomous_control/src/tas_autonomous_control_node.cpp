#include "control/control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);

    ROS_INFO("Automatic Control! Use the AUX switch on the controller to trigger this mode");

    while(ros::ok())
    {
       
        if(autonomous_control.cmd_linearVelocity>0)
        {
            //autonomous_control.control_servo.x = 1550; //1580 for Gertty; // ori: 1550 changed by hao//1575; changed by group 3
	    autonomous_control.control_servo.x = 1580;	
	    ROS_INFO("linearVelocity more than 0");
            ROS_INFO_STREAM("velocity: " << autonomous_control.cmd_linearVelocity );
            ROS_INFO_STREAM("angle: " << autonomous_control.cmd_steeringAngle );
        }
        else if(autonomous_control.cmd_linearVelocity<0)
        {
            autonomous_control.control_servo.x = 1300;
	    ROS_INFO("linearVelocity less than 0");
        }
        else
        {
	    if (autonomous_control.cmd_steeringAngle>1999)
	    {
		//autonomous_control.control_servo.x = 1550; //1580 for Gertty; // 1550; 1575 changed by group 3
		autonomous_control.control_servo.x = 1580; 		
		ROS_INFO("steeringAngle more than 1999");
		ROS_INFO_STREAM("velocity: " << autonomous_control.cmd_steeringAngle );
	    }
	    else if (autonomous_control.cmd_steeringAngle<1001)
	    {
		//autonomous_control.control_servo.x = 1550; //1580 for Gertty;// 1550; 1575 changed by group 3
		autonomous_control.control_servo.x = 1580;		
		ROS_INFO("steeringAngle less than 1001");
                ROS_INFO_STREAM("steeringAngle: " << autonomous_control.cmd_steeringAngle );
	    }
	    else
	    {
		autonomous_control.control_servo.x = 1500;
		ROS_INFO("just stand");
	    }
        }
        autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
	// changed by group 3 direction correction

        autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

   

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}

