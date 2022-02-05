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
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

static int motor_cmd_to_radsec;
static int encoder_ticks_to_rad;
turtlelib::Config q_old, q_new;
turtlelib::WheelPhi phi_old, phi_new;
turtlelib::Twist V_applied;
turtlelib::DiffDrive diff_drive;
static double dist, radius;
turtlelib::Vector2D u;
ros::Publisher wheel_cmd_pub, joint_state_pub;

void vel_sub_callback(const geometry_msgs::Twist& msg)
{
    
    V_applied.thetadot = msg.linear.x;
    V_applied.xdot = msg.linear.y;
    V_applied.ydot = msg.angular.z;

    u = diff_drive.InvKin(V_applied);

    nuturtlebot_msgs::WheelCommands wheel_cmd;
    wheel_cmd.left_velocity = u.x/motor_cmd_to_radsec;
    wheel_cmd.right_velocity = u.y/motor_cmd_to_radsec;

    wheel_cmd_pub.publish(wheel_cmd);

}

void sensor_data_callback(const nuturtlebot_msgs::SensorData& sensor_data)
{   
    sensor_msgs::JointState js_msg;

    js_msg.header.frame_id = "world";
    js_msg.name.push_back("red_wheel_left_joint");
    js_msg.name.push_back("red_wheel_right_joint");
    js_msg.position.push_back(sensor_data.left_encoder*encoder_ticks_to_rad);
    js_msg.position.push_back(sensor_data.right_encoder*encoder_ticks_to_rad);
    js_msg.velocity.push_back(sensor_data.left_encoder*motor_cmd_to_radsec);
    js_msg.velocity.push_back(sensor_data.right_encoder*motor_cmd_to_radsec);
    joint_state_pub.publish(js_msg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    
    nh.getParam("motor_cmd_to_radsec", motor_cmd_to_radsec);
    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);
    
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, vel_sub_callback);
    wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("/wheel_cmd", 10, 1);
    
    ros::Subscriber sensor_data_sub = nh.subscribe("/sensor_data", 10, sensor_data_callback);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10, 1);
    

    diff_drive = turtlelib::DiffDrive(dist, radius, phi_old, q_old);

    
    while(ros::ok())
    {


        loop_rate.sleep();
        
        
    }

    return 0;

}