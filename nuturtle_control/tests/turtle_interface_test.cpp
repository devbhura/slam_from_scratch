#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include "ros/console.h"

static nuturtlebot_msgs::WheelCommands wheel_cmd;
static sensor_msgs::JointState js_msg;

void vel_sub_callback(const nuturtlebot_msgs::WheelCommands& msg)
{
    wheel_cmd = msg;
}

void js_callback(const sensor_msgs::JointState& j_msg)
{
    js_msg = j_msg;

}

TEST_CASE("turtle interface test wheel_cmd", "[turtle interface test wheel_cmd]")
{
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);
    

    ros::Subscriber vel_sub = nh.subscribe("/wheel_cmd", 10, vel_sub_callback);
    // wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("/wheel_cmd", 10);
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ros::Subscriber sensor_data_sub = nh.subscribe("/sensor_data", 10, sensor_data_callback);
    // joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Subscriber js_sub = nh.subscribe("/joint_states", 10, js_callback);
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("/sensor_data", 10);
    

    SECTION("check cmd_vel translational","[translational cmd_vel]")
    {
        //initialize messages here
        geometry_msgs::Twist msg;
        msg.linear.x = 0.1;
        msg.linear.y = 0.0;
        msg.angular.z = 0.0;
        
        int i;
        while(1)
        {   
            i++;
            loop_rate.sleep();
            ros::spinOnce();
            if(i>100)
            {
                break;
            }
            cmd_vel_pub.publish(msg);
        }
        CHECK(wheel_cmd.left_velocity == Approx(126)); 
        CHECK(wheel_cmd.right_velocity == Approx(126));
    }

    SECTION("check cmd_vel rotational","[rotational cmd_vel]")
    {
        //initialize messages here
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.angular.z = 0.1;
        
        int i;
        while(1)
        {   
            i++;
            loop_rate.sleep();
            ros::spinOnce();
            if(i>100)
            {
                break;
            }
            cmd_vel_pub.publish(msg);
        }
        CHECK(wheel_cmd.left_velocity == Approx(-10.0)); 
        CHECK(wheel_cmd.right_velocity == Approx(10.0));
    }

    SECTION("check joint states ","[joint states]")
    {
        //initialize messages here
        nuturtlebot_msgs::SensorData sensor_data_msg;
        sensor_data_msg.left_encoder = 10;
        sensor_data_msg.right_encoder = 10;
        
        int i;
        while(1)
        {   
            sensor_pub.publish(sensor_data_msg);
            i++;
            loop_rate.sleep();
            ros::spinOnce();
            if(i>100)
            {
                break;
            }
            
        }
        CHECK(js_msg.position[0] == Approx(0.0153398)); 
        CHECK(js_msg.position[1] == Approx(0.0153398));
    } 

}
