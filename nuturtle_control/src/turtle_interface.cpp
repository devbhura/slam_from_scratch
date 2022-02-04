/// \file 
/// \brief enables control of the turtlebot

/**
 * PARAMETERS:
 * 
 * PUBLISHERS:
 * 
 * SUBSCRIBERS:
 * 
 * SERVICES:
 *  
 * 
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/WheelCommands"

void vel_sub_callback(const geometry_msgs::Twist &msg)
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh("~");
    
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, vel_sub_callback);
    ros::Publisher wheel_cmd = nh.advertise<nuturtlebot_msgs::WheelCommands>("/wheel_cmd", 10, 1);
    
    ros::Rate loop_rate(100);
    
    while(ros::ok())
    {


        loop_rate.sleep();
        
        
    }

    return 0;

}