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
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

static ros::Subscriber joint_state_sub;
static ros::Publisher odom_pub;
static nav_msgs::Odometry odom;
static turtlelib::DiffDrive diff_drive;
static double radius, dist;
static turtlelib::Config qhat;
static turtlelib::WheelPhi phi;

static geometry_msgs::TransformStamped transformStamped;

void joint_state_callback(const sensor_msgs::JointState& js_msg)
{
    

    phi.left_phi = js_msg.position[0];
    phi.right_phi = js_msg.position[1];

    
    qhat = diff_drive.ForwardKin(phi);
    diff_drive = turtlelib::DiffDrive(dist, radius, phi, qhat);

}

void publish_topics()
{
    odom.header.stamp = ros::Time::now();
    
    turtlelib::Config q_odom = diff_drive.getConfig();
    odom.pose.pose.position.x = q_odom.x;
    odom.pose.pose.position.y = q_odom.y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, q_odom.phi);
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom_pub.publish(odom);

    // TF 
    tf2_ros::TransformBroadcaster br;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = q_odom.x;
    transformStamped.transform.translation.y = q_odom.y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    br.sendTransform(transformStamped);
    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    std::string body_id, odom_id, wheel_left, wheel_right;

    nh.getParam("body_id", body_id);
    nh.param<std::string>("odom_id", odom_id , "odom");
    nh.getParam("wheel_left", wheel_left);
    nh.getParam("wheel_right", wheel_right);
    nh.getParam("dist", dist);
    nh.getParam("radius", radius);
    diff_drive = turtlelib::DiffDrive(dist, radius, phi, qhat);

    // subscribe to joint state
    joint_state_sub = nh.subscribe("/joint_states", 1, joint_state_callback);
    
    // Assign the publisher odom
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);


    // Odom message
    odom.header.frame_id = body_id;
    odom.child_frame_id = odom_id;

    // Transform definition
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = body_id;
    transformStamped.child_frame_id = odom_id;

    
    while(ros::ok())
    {

        publish_topics();
        loop_rate.sleep();
        
        
    }

    return 0;

}