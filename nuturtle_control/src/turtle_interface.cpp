/// \file 
/// \brief enables control of the turtlebot

/**
 * PARAMETERS:
    * motor_cmd_to_radsec (double): converts ticks to rad/sec
    * encoder_ticks_to_radsec (double): converts encoder ticks to rad/sec
    * dist (double): half the track width of the robot
    * radius (double): radius of the wheel of the robot 
 * PUBLISHERS:
    * wheel_cmd_pub: publishes to /wheel_cmd 
    * joint_state_pub: publishes to /joint_states
 * SUBSCRIBERS:
    * vel_sub: subscribes to /cmd_vel
    * sensor_data_sub: subscribes to /sensor_data
 * SERVICES:
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
static double encoder_ticks_to_rad;
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

/// \brief callback for cmd_vel subscriber
/// Input: 
/// \param msg - Twist being subscribed to
/// Output: Empty
void vel_sub_callback(const geometry_msgs::Twist& msg)
{
    
    V_applied.thetadot = msg.angular.z;
    V_applied.xdot = msg.linear.x;
    V_applied.ydot = msg.linear.y;

    u = diff_drive.InvKin(V_applied);
    
    wheel_cmd.left_velocity = (u.x/motor_cmd_to_radsec);
    wheel_cmd.right_velocity = (u.y/motor_cmd_to_radsec);

    if(wheel_cmd.left_velocity>256)
    {
        wheel_cmd.left_velocity = 256;
    }
    if(wheel_cmd.right_velocity>256)
    {
        wheel_cmd.right_velocity = 256;
    }
    
    if(wheel_cmd.right_velocity<-256)
    {
        wheel_cmd.right_velocity = -256;
    }
    if(wheel_cmd.left_velocity<-256)
    {
        wheel_cmd.left_velocity = -256;
    }

    wheel_cmd_pub.publish(wheel_cmd);

    
}

/// \brief callback for sensor_data subscriber
/// Input: 
/// \param msg - sensor_data being subscribed to
/// Output: Empty
void sensor_data_callback(const nuturtlebot_msgs::SensorData& sensor_data)
{   

    js_msg.header.stamp = ros::Time::now();
    js_msg.position = {sensor_data.left_encoder*encoder_ticks_to_rad, sensor_data.right_encoder*encoder_ticks_to_rad};
    ROS_INFO_ONCE("js_msg_vel_x %f", u.x); 
    ROS_INFO_ONCE("js_msg_vel_y %f", u.y); 

    js_msg.velocity = {u.x, u.y};

    joint_state_pub.publish(js_msg);
    
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
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    
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

    }

    return 0;

}