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
    * wall_marker: publishes the wall 
 * SERVICES:
    * reset: resets the position of the turtlebot3 to the original position
    * teleport: sets the position of the turtlebot3 to a new user specified position
 * SUBSCRIBERS:
    * sub_wheel: subscribe to wheel_cmd topic
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
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"


static double timestep;
static double x, x0;
static double y, yinit; 
static double theta, theta0;
static double obs_radius, x_length, y_length;
static visualization_msgs::MarkerArray obstacles_array, wall_array;
static std::vector<double> obstacles_x_arr, obstacles_y_arr, obstacles_theta_arr, wall_x_arr, wall_y_arr;
static ros::Publisher obstacle_marker, wall_marker, snsr_data;
static turtlelib::DiffDrive diff_drive;
static double motor_cmd_to_radsec;
static double encoder_ticks_to_rad;
static turtlelib::WheelPhi old_phi;
static int r;
static double dist, radius;
static turtlelib::Config q;
static nuturtlebot_msgs::SensorData sensor;

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

/// \brief
/// function that sets the wall array and publishes it
/// Input: None
/// Output: None
void walls()
{
    wall_array.markers.resize(4);
    
    // First wall
    wall_array.markers[0].header.frame_id = "world";
    wall_array.markers[0].header.stamp = ros::Time::now();
    wall_array.markers[0].id = 1;

    wall_array.markers[0].type = visualization_msgs::Marker::CUBE;
    wall_array.markers[0].action = visualization_msgs::Marker::ADD;

    wall_array.markers[0].pose.position.x = x_length/2;
    wall_array.markers[0].pose.position.y = 0.0;
    wall_array.markers[0].pose.position.z = 0.0;

    tf2::Quaternion q_obs;
    q_obs.setRPY(0, 0,0);

    wall_array.markers[0].pose.orientation.x = q_obs.x();
    wall_array.markers[0].pose.orientation.y = q_obs.y();
    wall_array.markers[0].pose.orientation.z = q_obs.z();
    wall_array.markers[0].pose.orientation.w = q_obs.w();

    wall_array.markers[0].scale.x = 0.25;
    wall_array.markers[0].scale.y = y_length;
    wall_array.markers[0].scale.z = 0.25;


    wall_array.markers[0].color.r = 0;
    wall_array.markers[0].color.g = 1;
    wall_array.markers[0].color.b = 0;
    wall_array.markers[0].color.a = 1;

    // Second wall
    wall_array.markers[1].header.frame_id = "world";
    wall_array.markers[1].header.stamp = ros::Time::now();
    wall_array.markers[1].id = 2;

    wall_array.markers[1].type = visualization_msgs::Marker::CUBE;
    wall_array.markers[1].action = visualization_msgs::Marker::ADD;

    wall_array.markers[1].pose.position.x = -x_length/2;
    wall_array.markers[1].pose.position.y = 0.0;
    wall_array.markers[1].pose.position.z = 0.0;

    wall_array.markers[1].pose.orientation.x = q_obs.x();
    wall_array.markers[1].pose.orientation.y = q_obs.y();
    wall_array.markers[1].pose.orientation.z = q_obs.z();
    wall_array.markers[1].pose.orientation.w = q_obs.w();

    wall_array.markers[1].scale.x = 0.25;
    wall_array.markers[1].scale.y = y_length;
    wall_array.markers[1].scale.z = 0.25;


    wall_array.markers[1].color.r = 0;
    wall_array.markers[1].color.g = 1;
    wall_array.markers[1].color.b = 0;
    wall_array.markers[1].color.a = 1;

    // Third walls
    wall_array.markers[2].header.frame_id = "world";
    wall_array.markers[2].header.stamp = ros::Time::now();
    wall_array.markers[2].id = 3;

    wall_array.markers[2].type = visualization_msgs::Marker::CUBE;
    wall_array.markers[2].action = visualization_msgs::Marker::ADD;

    wall_array.markers[2].pose.position.x = 0.0;
    wall_array.markers[2].pose.position.y = y_length/2;
    wall_array.markers[2].pose.position.z = 0.0;

    wall_array.markers[2].pose.orientation.x = q_obs.x();
    wall_array.markers[2].pose.orientation.y = q_obs.y();
    wall_array.markers[2].pose.orientation.z = q_obs.z();
    wall_array.markers[2].pose.orientation.w = q_obs.w();

    wall_array.markers[2].scale.x = x_length;
    wall_array.markers[2].scale.y = 0.25;
    wall_array.markers[2].scale.z = 0.25;


    wall_array.markers[2].color.r = 0;
    wall_array.markers[2].color.g = 1;
    wall_array.markers[2].color.b = 0;
    wall_array.markers[2].color.a = 1;

    // Fourth wall
    wall_array.markers[3].header.frame_id = "world";
    wall_array.markers[3].header.stamp = ros::Time::now();
    wall_array.markers[3].id = 4;

    wall_array.markers[3].type = visualization_msgs::Marker::CUBE;
    wall_array.markers[3].action = visualization_msgs::Marker::ADD;

    wall_array.markers[3].pose.position.x = 0.0;
    wall_array.markers[3].pose.position.y = -y_length/2;
    wall_array.markers[3].pose.position.z = 0.0;

    wall_array.markers[3].pose.orientation.x = q_obs.x();
    wall_array.markers[3].pose.orientation.y = q_obs.y();
    wall_array.markers[3].pose.orientation.z = q_obs.z();
    wall_array.markers[3].pose.orientation.w = q_obs.w();

    wall_array.markers[3].scale.x = x_length;
    wall_array.markers[3].scale.y = 0.25;
    wall_array.markers[3].scale.z = 0.25;


    wall_array.markers[3].color.r = 0;
    wall_array.markers[3].color.g = 1;
    wall_array.markers[3].color.b = 0;
    wall_array.markers[3].color.a = 1;


   wall_marker.publish(wall_array);
}

/// \brief callback for wheel_cmd subscriber
/// Input: 
/// \param input - wheel command being subscribed to
/// Output: Empty
void sub_wheel_callback(const nuturtlebot_msgs::WheelCommands& input)
{
    turtlelib::Vector2D u;

    u.x = input.left_velocity;
    u.y = input.right_velocity;

    u.x = u.x*motor_cmd_to_radsec;
    u.y = u.y*motor_cmd_to_radsec;

    old_phi.left_phi += u.x/r;
    old_phi.right_phi += u.y/r;

    q = diff_drive.ForwardKin(old_phi);

    diff_drive = turtlelib::DiffDrive(dist, radius, old_phi, q);

    sensor.left_encoder = int(old_phi.left_phi/encoder_ticks_to_rad);
    sensor.right_encoder = int(old_phi.right_phi/encoder_ticks_to_rad);

    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    ros::NodeHandle red("red");
    ros::NodeHandle obstacle("obstacle");
    
    // define variables
    // sensor_msgs::JointState joint_msg;
    
    // define subscribers
    ros::Subscriber sub_wheel = red.subscribe("/wheel_cmd", 1, sub_wheel_callback);

    // define services
    ros::ServiceServer reset = nh.advertiseService("reset",  reset_callback);
    ros::ServiceServer teleport = nh.advertiseService("teleport", teleport_callback);

    // Define Publishers
    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep", 100);
    // ros::Publisher joint_msg_pub = red.advertise<sensor_msgs::JointState>("joint_states", 1);
    obstacle_marker = obstacle.advertise<visualization_msgs::MarkerArray>("obs", 1, true);
    wall_marker = obstacle.advertise<visualization_msgs::MarkerArray>("wall", 1, true);
    snsr_data = red.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 10);

    // variables from the parameter
    
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
    nh.getParam("motor_cmd_to_radsec", motor_cmd_to_radsec);
    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("track_width", dist);
    nh.getParam("wheel_radius", radius);

    // Assign Diff Drive
    turtlelib::Config c;
    c.x = x0;
    c.y = yinit;
    c.phi = theta0;
    diff_drive = turtlelib::DiffDrive(dist, radius, old_phi, c);

    ros::Rate loop_rate(r);

    x = x0;
    y = yinit;
    theta = theta0;

    old_phi.left_phi = 0.0;
    old_phi.right_phi = 0.0;
    

    // Transform definition
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;


    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "red_base_footprint";
    transformStamped.transform.translation.x = c.x;
    transformStamped.transform.translation.y = c.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, c.phi);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();    
    
    walls();
    
    while(ros::ok())
    {
        
        std_msgs::UInt64 msg;
        msg.data = timestep;
        pub.publish(msg);

        
        timestep+=1;

        turtlelib::Config con = diff_drive.getConfig();
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = con.x;
        transformStamped.transform.translation.y = con.y;
        q.setRPY(0, 0, con.phi);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
        
        obstacles();
        
        snsr_data.publish(sensor);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}