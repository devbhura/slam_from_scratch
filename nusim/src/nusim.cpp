/// \file
/// \brief launches turtlebot3 and obstacles in rviz scene for visualization
/**
 * PARAMETERS:
    * timestep (double): tracks the current time step of the simulation
    * x0 (double): initial x position of the robot loaded
    * yinit (double): initial y position of the robot loaded
    * theta0 (double): initial theta orientation of the robot loaded
    * x (double): current x position of the robot 
    * y (double): current y position of the robot 
    * theta (double): current theta orientation of the robot 
    * obs_radius (double): radius of the obstacles
    * obstacles_array (visualization_msgs/MarkerArray): Creates a Marker Array for the obstacles
    * obstacles_x_arr (vector<double>): loads the x coordinates of all the obstacles in an array
    * obstacles_y_arr (vector<double>): loads the y coordinates of all the obstacles in an array
    * obstacles_theta_arr (vector<double>): loads the theta coordinates of all the obstacles in an array
    * loop_rate (ROS::Rate): defines the rate of the while loop
    * r (int): takes the rate from the parameters
 * BROADCASTERS:
    * br: Broadcasts transform from world frame to red_base_footprint frame 
 * PUBLISHERS:
    * obstacle_marker: publishes the obstacle marker 
    * pub: publishes the timestep
    * joint_msg_pub: publishes the joint states
 * SERVICES:
    *  reset: resets the position of the turtlebot3 to the original position
    *  teleport: sets the position of the turtlebot3 to a new user specified position
 * SUBSCRIBERS:
 */


#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include "nusim/teleport.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/console.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"


static double timestep;
static double x, x0;
static double y, yinit; 
static double theta, theta0;
static double obs_radius, x_length, y_length;
static visualization_msgs::MarkerArray obstacles_array, wall_array;
static std::vector<double> obstacles_x_arr, obstacles_y_arr, obstacles_theta_arr, wall_x_arr, wall_y_arr;
static ros::Publisher obstacle_marker, wall_marker;

/// \brief
/// callback for reset service that resets the positon of the robot to the original position
/// Input: None
/// Output: TriggerResponse
bool reset_callback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    /* 
     * 
    */
    timestep = 0.0;
    x = x0;
    y = yinit;
    theta = theta0;


    return response.success;
}

///\brief
/// callback for reset service that resets the positon of the robot to the original position
/// Input:
/// \param x - x position to set the robot to
/// \param y - y position to set the robot to
/// \param theta - theta: theta orientation to set the robot to
/// Output: teleportResponse
bool teleport_callback(nusim::teleport::Request& input, nusim::teleport::Response& response)
{

    x = input.x;
    y = input.y;
    theta = input.theta;
    return true;

}

///\brief
/// function that sets the obstacle marker array and publishes
/// Input: None
/// Output: None
void obstacles()
{   
    

    obstacles_array.markers.resize(obstacles_x_arr.size());

    for (int i = 0; i<obstacles_x_arr.size(); i++)
    {
        
        obstacles_array.markers[i].header.frame_id = "world";
        obstacles_array.markers[i].header.stamp = ros::Time::now();
        obstacles_array.markers[i].id = i;

        obstacles_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
        obstacles_array.markers[i].action = visualization_msgs::Marker::ADD;

        obstacles_array.markers[i].pose.position.x = obstacles_x_arr[i];
        obstacles_array.markers[i].pose.position.y = obstacles_y_arr[i];
        obstacles_array.markers[i].pose.position.z = 0.0;

        tf2::Quaternion q_obs;
        q_obs.setRPY(0, 0, obstacles_theta_arr[i]);

        obstacles_array.markers[i].pose.orientation.x = q_obs.x();
        obstacles_array.markers[i].pose.orientation.y = q_obs.y();
        obstacles_array.markers[i].pose.orientation.z = q_obs.z();
        obstacles_array.markers[i].pose.orientation.w = q_obs.w();

        obstacles_array.markers[i].scale.x = 2*obs_radius;
        obstacles_array.markers[i].scale.y = 2*obs_radius;
        obstacles_array.markers[i].scale.z = 0.25;


        obstacles_array.markers[i].color.r = 1;
        obstacles_array.markers[i].color.g = 0;
        obstacles_array.markers[i].color.b = 0;
        obstacles_array.markers[i].color.a = 1;

    }

   obstacle_marker.publish(obstacles_array);

}

// void walls()
// {
//     wall_x_arr.resize(4);
//     wall_x_arr.at(0) = x_length/2; 
//     wall_x_arr.at(1) = -x_length/2; 
//     wall_x_arr.at(2) = 0; 
//     wall_x_arr.at(3) = 0; 

//     wall_y_arr.resize(4);
//     wall_y_arr.at(0) = 0;
//     wall_y_arr.at(1) = 0;
//     wall_y_arr.at(2) = y_length/2;
//     wall_y_arr.at(3) = -y_length/2;

//     wall_array.markers.resize(4);

//     for (int i = 0; i<2; i++)
//     {
        
//         wall_array.markers[i].header.frame_id = "world";
//         wall_array.markers[i].header.stamp = ros::Time::now();
//         wall_array.markers[i].id = i;

//         wall_array.markers[i].type = visualization_msgs::Marker::CUBE;
//         wall_array.markers[i].action = visualization_msgs::Marker::ADD;

//         wall_array.markers[i].pose.position.x = wall_x_arr[i];
//         wall_array.markers[i].pose.position.y = wall_y_arr[i];
//         wall_array.markers[i].pose.position.z = 0.0;

//         tf2::Quaternion q_obs;
//         q_obs.setRPY(0, 0,0);

//         wall_array.markers[i].pose.orientation.x = q_obs.x();
//         wall_array.markers[i].pose.orientation.y = q_obs.y();
//         wall_array.markers[i].pose.orientation.z = q_obs.z();
//         wall_array.markers[i].pose.orientation.w = q_obs.w();

//         wall_array.markers[i].scale.x = 2*wall_y_arr[i] + 0.25;
//         wall_array.markers[i].scale.y = 2*wall_y_arr[i] + 0.25;
//         wall_array.markers[i].scale.z = 0.25;


//         wall_array.markers[i].color.r = 0;
//         wall_array.markers[i].color.g = 1;
//         wall_array.markers[i].color.b = 0;
//         wall_array.markers[i].color.a = 1;

//     }

//    wall_marker.publish(wall_array);
// }

bool sub_wheel_callback(nuturtlebot_msgs::WheelCommands& input)
{
    

    return true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    ros::NodeHandle red("red");
    ros::NodeHandle obstacle("obstacle");
    
    // define variables
    sensor_msgs::JointState joint_msg;
    
    // define subscribers
    ros::Subscriber sub_wheel = red.subscribe("/wheel_cmd", 1, sub_wheel_callback);

    // define services
    ros::ServiceServer reset = nh.advertiseService("reset",  reset_callback);
    ros::ServiceServer teleport = nh.advertiseService("teleport", teleport_callback);

    // Define Publishers
    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep", 100);
    ros::Publisher joint_msg_pub = red.advertise<sensor_msgs::JointState>("joint_states", 1);
    obstacle_marker = obstacle.advertise<visualization_msgs::MarkerArray>("obs", 1, true);
    wall_marker = obstacle.advertise<visualization_msgs::MarkerArray>("wall", 1, true);


    // variables from the parameter
    int r;
    timestep = 0;
    nh.param("rate", r, 500);
    nh.param("x0",x0,0.0);
    nh.param("y0",yinit,0.0);
    nh.param("theta0",theta0,0.0);
    nh.getParam("obstacles_x_arr", obstacles_x_arr);
    nh.getParam("obstacles_y_arr", obstacles_y_arr);
    nh.getParam("obstacles_theta_arr", obstacles_theta_arr);
    nh.param("obs_radius",obs_radius,0.025);
    nh.param("x_length", x_length, 5.0);
    nh.param("y_length", y_length, 5.0);

    


    ros::Rate loop_rate(r);

    x = x0;
    y = yinit;
    theta = theta0;
    

    // Transform definition
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;


    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "red_base_footprint";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    joint_msg.header.frame_id = "world";
    joint_msg.name.push_back("red_wheel_left_joint");
    joint_msg.name.push_back("red_wheel_right_joint");
    joint_msg.position.push_back(0.0);
    joint_msg.position.push_back(0.0);
    joint_msg_pub.publish(joint_msg);
    
    
    // walls();
    
    while(ros::ok())
    {
        
        std_msgs::UInt64 msg;
        msg.data = timestep;
        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        timestep+=1;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        q.setRPY(0, 0, theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
        ROS_DEBUG("x0: %f", x0);
        
        obstacles();
        
    

    }

    return 0;

}