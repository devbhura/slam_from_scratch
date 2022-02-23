#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include "nuturtle_control/set_pose.h"
#include "ros/console.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


static nav_msgs::Odometry odom_state;

// odom subscriber callback
void odom_sub_callback(const nav_msgs::Odometry& msg)
{
    odom_state = msg;

}


TEST_CASE("odom test", "[odom test]")
{
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_sub_callback);
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    SECTION("tf2 listener","[tf2 listener]")
    {
        //initialize messages here
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        int  i = 0;
        geometry_msgs::TransformStamped transformStamped;
        while (1){

            i++;
            
            try{
                transformStamped = tfBuffer.lookupTransform("odom", "blue_base_footprint",
                                ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
                }
            
            if(i>100)
            {
                break;
            }
        }
        
        loop_rate.sleep();
        ros::spinOnce();
        CHECK(transformStamped.transform.translation.x == Approx(0.0)); 
        CHECK(transformStamped.transform.translation.y == Approx(0.0));
        
    }

    SECTION("set pose","[set_pose]")
    {
        //initialize messages here
        ros::ServiceClient client = nh.serviceClient<nuturtle_control::set_pose>("/set_pose");
        nuturtle_control::set_pose srv;
        srv.request.x = 0.1;
        srv.request.y = 0.1;
        srv.request.phi = 0.1;

        while(1)
        {

            if(client.call(srv))
            {
                client.call(srv);
                break;
            }
            

        }
        
        ros::Duration(5).sleep();
        loop_rate.sleep();
        ros::spinOnce();
        CHECK(odom_state.pose.pose.position.x == Approx(0.1)); 
        CHECK(odom_state.pose.pose.position.y == Approx(0.1));
        
    }
    

}