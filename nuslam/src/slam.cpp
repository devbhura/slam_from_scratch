/// \file
/// \brief 
/**
 * PARAMETERS:
    * 
    * 
 * BROADCASTERS:
    * 
 * PUBLISHERS:
    * 
 * SERVICES:
    * 
 * SUBSCRIBERS:
    * 
 */

#include <armadillo>
#include "ros/ros.h"
#include <string>
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/MarkerArray.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include "nuslam/ekf.hpp"
#include <iostream>

static ros::Subscriber joint_state_sub, fake_sensor_sub;
static ros::Publisher odom_pub;
static nav_msgs::Odometry odom;
static turtlelib::DiffDrive diff_drive;
static double radius, dist;
static turtlelib::Config qhat, c;
static turtlelib::WheelPhi phi;
static geometry_msgs::TransformStamped transformStamped;
static turtlelib::Twist twist;
static std::vector<std::vector<double>> measurement;
static slam::ekf ekf_slam;

void initialize();

/// \brief
/// callback for joint_state subscriber
/// Input: 
/// \param js_msg - joint state being subscribed to
/// Output: Empty
void joint_state_callback(const sensor_msgs::JointState& js_msg)
{
    

    phi.left_phi = js_msg.position.at(0);
    phi.right_phi = js_msg.position.at(1);

    
    qhat = diff_drive.ForwardKin(phi);
    diff_drive = turtlelib::DiffDrive(dist, radius, phi, qhat);
    twist.xdot = c.x - qhat.x;
    twist.ydot = c.y - qhat.y;
    twist.thetadot = c.phi - qhat.phi;
    c = qhat;

}

/// \brief function to update odom and broadcaster
/// Input: None
/// Output: None
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
    
    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.linear.y = twist.ydot;
    odom.twist.twist.angular.z = twist.thetadot;

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

/// \brief
/// callback for fake sensor subscriber
/// Input: 
/// \param obstacle_msg - Marker Array message being subscribed to
/// Output: Empty
void fake_sensor_callback(const visualization_msgs::MarkerArray& obstacle_msg)
{
    int obstacle_size = obstacle_msg.markers.size();

    measurement.resize(obstacle_size, std::vector<double>(2));

    for(int i = 0; i<obstacle_size; i++)
    {
        double x, y, r, phi;
        x = obstacle_msg.markers.at(i).pose.position.x;
        y = obstacle_msg.markers.at(i).pose.position.y;

        r = sqrt(pow(x,2)+pow(y,2));
        phi = atan2(y,x);

        measurement[i][0] = r;
        measurement[i][1] = phi;
    }
}

void slam_timer_callback(const ros::TimerEvent&)
{

}

void initialize()
{
    //initialize ekf slam object
    turtlelib::Config con = diff_drive.getConfig();
    arma::Mat<double> q_0; 
    q_0 = {con.phi, con.x, con.y};
    arma::Mat<double> m_0;
    int m_size = measurement.size();

    for(int i=0; i<m_size; i++)
    {
        double r, phi_m; 
        r = measurement[i][0];
        phi_m = measurement[i][1];
        m_0 = {(con.x + (r*cos(phi_m + double(con.phi)))), (con.y + (r*sin(phi_m + double(con.phi)))) };
        q_0 = join_cols(q_0, m_0);
    }

    arma::Mat<double> Sigma_0q = arma::zeros(3,3);
    arma::Mat<double> Sigma_0m = arma::ones(2*m_size,2*m_size);
    Sigma_0m = Sigma_0m*200; 
    arma::Mat<double> zeros2n_3 = arma::zeros(2*m_size,3);
    arma::Mat<double> zeros3_2n = arma::zeros(3,2*m_size);

    arma::Mat<double> Sigma_0 = join_rows(Sigma_0q, zeros3_2n);
    Sigma_0 = join_cols(Sigma_0,join_rows(zeros2n_3,Sigma_0m)); 
    ekf_slam.ekf_size(m_size);
    ekf_slam.initial_state(q_0, Sigma_0); 

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
    nh.param("x0",x0,0.0);
    nh.param("y0",yinit,0.0);
    nh.param("theta0",theta0,0.0);

    c.x = x0;
    c.y = yinit;
    c.phi = theta0;

    diff_drive = turtlelib::DiffDrive(dist, radius, phi, c);

    
    // subscribe to joint state
    joint_state_sub = nh.subscribe("joint_states", 1, joint_state_callback);
    fake_sensor_sub = nh.subscribe("obstacle/fake_sensor", 5, fake_sensor_callback);
    ros::Timer slam_timer = nh.createTimer(ros::Duration(0.2), slam_timer_callback);

    // Assign the publisher odom
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    // Odom message
    odom.header.frame_id = body_id;
    odom.child_frame_id = odom_id;

    // Transform definition
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odom_id;
    transformStamped.child_frame_id = "blue_base_footprint";

    tf2_ros::TransformBroadcaster br;

    while(ros::ok())
    {
        

        publish_topics();
        odom_pub.publish(odom);
        br.sendTransform(transformStamped);
        ros::spinOnce();
        loop_rate.sleep();
        
        
    }

    return 0;

}


