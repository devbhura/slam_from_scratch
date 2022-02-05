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
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    int body_id;
    std::string odom_id;

    nh.getParam("body_id", body_id);
    nh.param<std::string>("odom_id", odom_id , "odom");

    
    while(ros::ok())
    {


        loop_rate.sleep();
        
        
    }

    return 0;

}