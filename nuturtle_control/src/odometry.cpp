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
#include "nuturtle_control/set_pose.h"

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
    

    phi.left_phi = js_msg.position.at(0);
    phi.right_phi = js_msg.position.at(1);

    
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
    

    // TF 
    

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = q_odom.x;
    transformStamped.transform.translation.y = q_odom.y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    


}

bool set_pose_callback(nuturtle_control::set_pose::Request& input, nuturtle_control::set_pose::Response& response)
{

    qhat.x = input.x;
    qhat.y = input.y;
    qhat.phi = input.phi;
    phi.left_phi = 0.0;
    phi.right_phi = 0.0;
    diff_drive = turtlelib::DiffDrive(dist, radius, phi, qhat);
    return true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);
    ros::NodeHandle nusim("~nusim");

    std::string body_id, odom_id, wheel_left, wheel_right;

    double x0, yinit, theta0;

    nh.getParam("body_id", body_id);
    nh.param<std::string>("odom_id", odom_id , "odom");
    nh.getParam("wheel_left", wheel_left);
    nh.getParam("wheel_right", wheel_right);
    nh.getParam("dist", dist);
    nh.getParam("radius", radius);
    nusim.param("x0",x0,-0.6);
    nusim.param("y0",yinit,0.8);
    nusim.param("theta0",theta0,0.0);

    turtlelib::Config c;
    c.x = x0;
    c.y = yinit;
    c.phi = theta0;

    diff_drive = turtlelib::DiffDrive(dist, radius, phi, c);

    // subscribe to joint state
    joint_state_sub = nh.subscribe("/joint_states", 1, joint_state_callback);
    
    // Assign the publisher odom
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    //Define services
    ros::ServiceServer set_pose = nh.advertiseService("set_pose", set_pose_callback);

    // Odom message
    odom.header.frame_id = body_id;
    odom.child_frame_id = odom_id;

    // Transform definition
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odom_id;
    transformStamped.child_frame_id = "blue_base_footprint";

    
    while(ros::ok())
    {
        

        publish_topics();
        odom_pub.publish(odom);
        tf2_ros::TransformBroadcaster br;
        br.sendTransform(transformStamped);
        ros::spinOnce();
        loop_rate.sleep();
        
        
    }

    return 0;

}