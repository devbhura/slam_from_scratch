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


static double timestep;
static double x, x0;
static double y, yinit; 
static double theta, theta0;
static double obs_radius;
visualization_msgs::MarkerArray obstacles_array;
std::vector<double> obstacles_x_arr, obstacles_y_arr, obstacles_theta_arr;
ros::Publisher obstacle_marker;

bool reset_callback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    timestep = 0.0;
    x = x0;
    y = yinit;
    theta = theta0;


    return true;
}

bool teleport_callback(nusim::teleport::Request& input, nusim::teleport::Response& response)
{

    x = input.x;
    y = input.y;
    theta = input.theta;
    return true;

}

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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    ros::NodeHandle red("red");
    ros::NodeHandle obstacle("obstacle");
    
    // define variables
    sensor_msgs::JointState joint_msg;
    ros::Rate loop_rate(10);

    // define services
    ros::ServiceServer reset = nh.advertiseService("reset",  reset_callback);
    ros::ServiceServer teleport = nh.advertiseService("teleport", teleport_callback);

    // Define Publishers
    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep", 100);
    ros::Publisher joint_msg_pub = red.advertise<sensor_msgs::JointState>("joint_states", 1);
    obstacle_marker = obstacle.advertise<visualization_msgs::MarkerArray>("obs", 1);
    


    // variables from the parameter
    int r;
    timestep = 0;
    nh.param("rate", r, 500);
    nh.param("timestep", timestep, 10.0);
    nh.param("x0",x0,0.0);
    nh.param("y0",yinit,0.0);
    nh.param("theta0",theta0,0.0);
    nh.getParam("obstacles_x_arr", obstacles_x_arr);
    nh.getParam("obstacles_y_arr", obstacles_y_arr);
    nh.getParam("obstacles_theta_arr", obstacles_theta_arr);
    nh.param("obs_radius",obs_radius,0.025);

    x = x0;
    y = yinit;
    theta = theta0;
    

    // Transform definition
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;


    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "red_base_link";
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
        
    
    obstacles();
    while(ros::ok())
    {
        
        std_msgs::UInt64 msg;
        msg.data = timestep;
        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        timestep+=1;


        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        q.setRPY(0, 0, theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

    }

    return 0;

}