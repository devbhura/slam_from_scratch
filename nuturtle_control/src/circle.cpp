/// \file 
/// \brief publishes odometry messages and the odometry transform

/**
 * PARAMETERS:
    * freq - sets the frequency of the node
 * PUBLISHERS:
    * vel_pub (geometry_msgs/Twist) - publishes the velocity to cmd_vel
 * SERVICES:
    * control - set the velocity and radius for the circular motion of the robot
    * reverse - reverse the direction  of motion of the robot
    * stop - make the robot stop 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "nuturtle_control/control.h"

static double radius, velocity;
static ros::Publisher vel_pub;
static int freq;
static geometry_msgs::Twist vel_cmd;

/// \brief
/// callback for control service that makes the robot move in a circle
/// Input: 
/// \param control - sets the velocity and radius for the circle
/// Output: Empty
bool control_callback(nuturtle_control::control::Request& control, nuturtle_control::control::Response& response)
{
    velocity  = control.velocity;
    radius = control.radius;

    vel_cmd.linear.x = velocity;
    vel_cmd.angular.z = velocity/radius;

    return true;
}

/// \brief
/// callback for reversing the direction of motion of the robot
/// Input: Empty
/// Output: Empty
bool reverse_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse &resp)
{
    vel_cmd.linear.x = -vel_cmd.linear.x ;
    vel_cmd.angular.z = -vel_cmd.angular.z;
    
    return true;
}

/// \brief
/// callback to stop the motion of the robot
/// Input: Empty
/// Output: Empty
bool stop_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse &resp)
{
    vel_cmd.linear.x = 0.0;
    vel_cmd.angular.z = 0.0;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh("~");
    nh.param("frequency", freq, 100);

    ros::Rate loop_rate(freq);

    // Publish to cmd_vel
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Services
    ros::ServiceServer control = nh.advertiseService("control",  control_callback);
    ros::ServiceServer reverse = nh.advertiseService("reverse", reverse_callback);
    ros::ServiceServer stop = nh.advertiseService("stop", stop_callback);

    while(ros::ok())
    {
        vel_pub.publish(vel_cmd);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}