/// \file 
/// \brief publishes odometry messages and the odometry transform

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
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "nuturtle_control/control.h"

static double radius, velocity;
static ros::Publisher vel_pub;
static int freq;
static geometry_msgs::Twist vel_cmd;

bool control_callback(nuturtle_control::control::Request& control, nuturtle_control::control::Response& response)
{
    velocity  = control.velocity;
    radius = control.radius;

    vel_cmd.linear.x = velocity;
    vel_cmd.angular.z = velocity/radius;

    return true;
}

bool reverse_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse &resp)
{
    vel_cmd.linear.x = -vel_cmd.linear.x ;

    return true;
}

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
    ros::ServiceServer control = nh.advertiseService("reset",  control_callback);
    ros::ServiceServer reverse = nh.advertiseService("teleport", reverse_callback);
    ros::ServiceServer stop = nh.advertiseService("stop", stop_callback);

    while(ros::ok())
    {
        
        ros::spinOnce();

        loop_rate.sleep();
        vel_pub.publish(vel_cmd);

    }

    return 0;
}