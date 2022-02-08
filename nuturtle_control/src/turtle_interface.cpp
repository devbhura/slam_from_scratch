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
#include "ros/console.h"

static double motor_cmd_to_radsec;
static int encoder_ticks_to_rad;
static turtlelib::Config q_old, q_new;
static turtlelib::WheelPhi phi_old, phi_new;
static turtlelib::Twist V_applied;
static turtlelib::DiffDrive diff_drive;
static double dist, radius;
static turtlelib::Vector2D u;
static ros::Publisher wheel_cmd_pub, joint_state_pub;
static nuturtlebot_msgs::WheelCommands wheel_cmd;
static sensor_msgs::JointState js_msg;
static int saved_left_vel_tick = 0;
static int saved_right_vel_tick = 0;

void vel_sub_callback(const geometry_msgs::Twist& msg)
{
    
    V_applied.thetadot = msg.angular.z;
    V_applied.xdot = msg.linear.x;
    V_applied.ydot = msg.linear.y;

    u = diff_drive.InvKin(V_applied);
    // ROS_INFO_STREAM("calculated u.x: %f" << u.x);
    // ROS_INFO_STREAM("calculated u.y: %f" << u.y);
    // ROS_INFO_STREAM("motor_cmd_2_rs: %f" << motor_cmd_to_radsec);

    wheel_cmd.left_velocity = int(u.x/motor_cmd_to_radsec);
    wheel_cmd.right_velocity = int(u.y/motor_cmd_to_radsec);

    // ROS_INFO_STREAM("calculated wheel_cmd.left: %d" << wheel_cmd.left_velocity);
    // ROS_INFO_STREAM("calculated wheel_cmd.right: %d" << wheel_cmd.right_velocity);

}

void sensor_data_callback(const nuturtlebot_msgs::SensorData& sensor_data)
{   

    js_msg.header.stamp = ros::Time::now();
    double js_pos1 = double(sensor_data.left_encoder - saved_left_vel_tick)/double(encoder_ticks_to_rad);
    double js_pos2 = double(sensor_data.right_encoder - saved_right_vel_tick)/double(encoder_ticks_to_rad);
    js_msg.position = {js_pos1, js_pos2};
    double js_vel1 = double(sensor_data.left_encoder - saved_left_vel_tick)*double(motor_cmd_to_radsec);
    double js_vel2 = double(sensor_data.right_encoder - saved_right_vel_tick)*double(motor_cmd_to_radsec);
    js_msg.velocity = {js_vel1, js_vel2};
    

    ROS_INFO_STREAM("saved_right_vel_tick: %d" << saved_right_vel_tick);
    ROS_INFO_STREAM("sensor_data: %d" << sensor_data.right_encoder);
    ROS_INFO_STREAM("diff: %d" << (sensor_data.right_encoder - saved_right_vel_tick));
    ROS_INFO_STREAM("js_msg.velocity: %f" << js_vel1);
    ROS_INFO_STREAM("motor_cmd_to_radsec: %f" << motor_cmd_to_radsec);
    saved_left_vel_tick = sensor_data.left_encoder;
    saved_right_vel_tick = sensor_data.right_encoder;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    
    nh.getParam("motor_cmd_to_radsec", motor_cmd_to_radsec);
    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("dist", dist);
    nh.getParam("radius", radius);
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, vel_sub_callback);
    wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("/wheel_cmd", 10);
    
    ros::Subscriber sensor_data_sub = nh.subscribe("/sensor_data", 10, sensor_data_callback);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    
    js_msg.header.frame_id = "world";
    js_msg.name.push_back("red_wheel_left_joint");
    js_msg.name.push_back("red_wheel_right_joint");
    js_msg.position.push_back(0.0);
    js_msg.position.push_back(0.0);
    
    

    diff_drive = turtlelib::DiffDrive(dist, radius, phi_old, q_old);

    
    while(ros::ok())
    {

        ros::spinOnce();

        loop_rate.sleep();
        
        wheel_cmd_pub.publish(wheel_cmd);
        joint_state_pub.publish(js_msg);

    }

    return 0;

}