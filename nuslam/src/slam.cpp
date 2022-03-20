/// \file
/// \brief runs EKF slam
/**
 * PARAMETERS:
    * body_id (string): name of the body frame of the robot
    * odom_id (string): name of the odom frame
    * wheel_left (string): name of the left wheel joint
    * wheel_right (string): name of the right wheel joint
    * dist (double): half the track width of the robot
    * radius (double): radius of the wheel of the robot
    * x0 (double): initial x position
    * y0 (double): initial y position
    * theta0 (double): initial angular orientation
    * obs_radius (double): radius of the obstacle
 * BROADCASTERS:
    * br: transform from world to blue_base_footprint
    * map_br: transform from map to odom 
    * green_br: transform from odom to green_base_footprint
 * PUBLISHERS:
    * odom_pub: publishes odometry
    * green_path_pub: publishes the path of the green robot
    * blue_path_pub: publishes the path of the blue robot
    * green_sensor_pub: publishes the landmarks as seen in slam
    * 
 * SERVICES:
    * 
 * SUBSCRIBERS:
    * joint_state_sub: subscribes to joint_states topic
    * fake_sensor_sub: subscribes to landmarks topic from landmarks.cpp
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
#include <ros/console.h>
#include "nav_msgs/Path.h"

static ros::Subscriber joint_state_sub, fake_sensor_sub;
static ros::Publisher odom_pub, green_path_pub, green_sensor_pub;
static nav_msgs::Odometry odom;
static turtlelib::DiffDrive diff_drive;
static double radius, dist;
static turtlelib::Config qhat, c, slam_config;
static turtlelib::WheelPhi phi;
static geometry_msgs::TransformStamped transformStamped, map2odom_trans, odom2green_trans;
static turtlelib::Twist twist;
static std::vector<std::vector<double>> measurement;
static slam::ekf ekf_slam;
static bool slam_flag = false; 
static nav_msgs::Path green_path, blue_path; 
static visualization_msgs::MarkerArray green_fake_sensor;
static double obs_radius;  

void initialize();

/// \brief
/// callback for joint_state subscriber
/// Input: 
/// \param js_msg - joint state being subscribed to
/// Output: Empty
void joint_state_callback(const sensor_msgs::JointState& js_msg)
{
    ROS_INFO_ONCE("js_msg_vel_x %d", int((js_msg.velocity).size()));
    
    phi.left_phi = js_msg.position.at(0);
    phi.right_phi = js_msg.position.at(1);
    // double lef_vel = js_msg.velocity.at(0); 
    // double right_vel = js_msg.velocity.at(1); 
    // ROS_WARN("%f %f",lef_vel,right_vel);
    // turtlelib::WheelPhi wheel_speed{lef_vel, right_vel}; 
    // ROS_WARN("N");

    // twist = diff_drive.getTwist(wheel_speed); 
    twist = diff_drive.getTwist(phi); 
    // ROS_INFO_STREAM("Twist from diff_drive" << twist); 
    qhat = diff_drive.ForwardKin(phi);
    
    diff_drive = turtlelib::DiffDrive(dist, radius, phi, qhat);
    
    // twist.xdot = c.x - qhat.x;
    // twist.ydot = c.y - qhat.y;
    // twist.thetadot = c.phi - qhat.phi;
    // c = qhat;

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

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = q_odom.x;
    pose.pose.position.y = q_odom.y; 
    pose.pose.position.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, q_odom.phi);
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
    
    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.linear.y = twist.ydot;
    odom.twist.twist.angular.z = twist.thetadot;
    blue_path.header.stamp = ros::Time::now();
    blue_path.header.frame_id = "world";
    blue_path.poses.push_back(pose);

    // TF 
    

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = q_odom.x;
    transformStamped.transform.translation.y = q_odom.y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    // Path
   
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

    // ROS_INFO_STREAM("measurement as measured"); 
    for(int i = 0; i<obstacle_size; i++)
    {
        double x, y, r, phi;
        x = obstacle_msg.markers.at(i).pose.position.x;
        y = obstacle_msg.markers.at(i).pose.position.y;

        r = sqrt(pow(x,2)+pow(y,2));
        phi = atan2(y,x);

        measurement[i][0] = r;
        measurement[i][1] = phi;
        
        // ROS_INFO_STREAM(measurement[i][0]); 
        // ROS_INFO_STREAM(measurement[i][1]); 
    }
}

/// \brief
/// timer callback
/// It runs the slam algorithm at the specified frequency. 
void slam_timer_callback(const ros::TimerEvent&)
{
    if(slam_flag)
    {
    std::vector<std::vector<double>> measurement_temp;
    measurement_temp = measurement;  
    int m_size = measurement_temp.size();
    green_fake_sensor.markers.resize(m_size);


    for(int i = 0; i<m_size; i++)
    {   
        arma::Mat<double> m(2,1);
        m(0) = measurement_temp[i][0];
        m(1) =  measurement_temp[i][1]; 
        int id = ekf_slam.landmark_association(m); 

        // if(id<0)
        // {
        //     break; 
        // }
        turtlelib::Twist u = twist;
        // ROS_INFO_STREAM("Twist"<<u); 
        arma::Mat<double> q_predict =  ekf_slam.predict_q(u);
        arma::Mat<double> A = ekf_slam.calc_A(u); 
        ekf_slam.predict();
        ekf_slam.calc_H(id);
        
        arma::Mat<double> q = ekf_slam.update(m);
        // ROS_INFO_STREAM("robot state" << q); 
        slam_config.phi = q(0);
        slam_config.x = q(1);
        slam_config.y = q(2);

        // Publish Green Markers
        green_fake_sensor.markers[id].header.frame_id = "map";
        green_fake_sensor.markers[id].header.stamp = ros::Time::now();
        green_fake_sensor.markers[id].id = id;

        green_fake_sensor.markers[id].type = visualization_msgs::Marker::CYLINDER;
        green_fake_sensor.markers[id].action = visualization_msgs::Marker::ADD;

        green_fake_sensor.markers[id].pose.position.x = q(3+2*id);
        green_fake_sensor.markers[id].pose.position.y = q(4+2*id);
        green_fake_sensor.markers[id].pose.position.z = 0.0;

        tf2::Quaternion q_obs;
        q_obs.setRPY(0, 0, 0);

        green_fake_sensor.markers[id].pose.orientation.x = q_obs.x();
        green_fake_sensor.markers[id].pose.orientation.y = q_obs.y();
        green_fake_sensor.markers[id].pose.orientation.z = q_obs.z();
        green_fake_sensor.markers[id].pose.orientation.w = q_obs.w();

        green_fake_sensor.markers[id].scale.x = 2*obs_radius;
        green_fake_sensor.markers[id].scale.y = 2*obs_radius;
        green_fake_sensor.markers[id].scale.z = 0.25;


        green_fake_sensor.markers[id].color.r = 0;
        green_fake_sensor.markers[id].color.g = 1;
        green_fake_sensor.markers[id].color.b = 0;
        green_fake_sensor.markers[id].color.a = 1;

        green_sensor_pub.publish(green_fake_sensor); 

        // 

        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = slam_config.x;
        pose.pose.position.y = slam_config.y; 
        pose.pose.position.z = 0.0;

        // ROS_INFO_STREAM("slam_config.x"<< slam_config.x); 
        // ROS_INFO_STREAM("slam_config.y"<< slam_config.y);
        // ROS_INFO_STREAM("slam_config.phi"<< slam_config.phi);


        tf2::Quaternion quat;
        quat.setRPY(0, 0, slam_config.phi);
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();
        
        green_path.header.stamp = ros::Time::now();
        green_path.header.frame_id = "map";
        green_path.poses.push_back(pose);
    }

    green_path_pub.publish(green_path); 

    } 
}

/// \brief
/// publish slam
/// Publishes all transfroms and results from slam 
void publish_slam()
{
    turtlelib::Config odom_q = diff_drive.getConfig(); 
    turtlelib::Transform2D odom2green_tf{turtlelib::Vector2D{odom_q.x, odom_q.y}, odom_q.phi}; 
    
    // odom to green base footprint transform
    odom2green_trans.header.stamp = ros::Time::now();
    odom2green_trans.transform.translation.x = odom_q.x;
    odom2green_trans.transform.translation.y = odom_q.y;
    odom2green_trans.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, odom_q.phi);
    odom2green_trans.transform.rotation.x = quat.x();
    odom2green_trans.transform.rotation.y = quat.y();
    odom2green_trans.transform.rotation.z = quat.z();
    odom2green_trans.transform.rotation.w = quat.w();

    turtlelib::Transform2D map2green_tf{turtlelib::Vector2D{slam_config.x, slam_config.y}, slam_config.phi};
    turtlelib::Transform2D map2odom_tf; 
    map2odom_tf = map2green_tf*(odom2green_tf.inv()); 

    turtlelib::Vector2D map2odom_vec = map2odom_tf.translation(); 
    double map2odom_ang = map2odom_tf.rotation(); 
    map2odom_trans.header.stamp = ros::Time::now();
    map2odom_trans.transform.translation.x = map2odom_vec.x;
    map2odom_trans.transform.translation.y = map2odom_vec.y;
    map2odom_trans.transform.translation.z = 0.0;
    
    quat.setRPY(0, 0, map2odom_ang);
    map2odom_trans.transform.rotation.x = quat.x();
    map2odom_trans.transform.rotation.y = quat.y();
    map2odom_trans.transform.rotation.z = quat.z();
    map2odom_trans.transform.rotation.w = quat.w();

    
}

/// \brief
/// Initalize slam
/// Initializes slam once landmarks are seen 
void initialize()
{
    //initialize ekf slam object
    turtlelib::Config con = diff_drive.getConfig();
    int m_size = measurement.size();
    arma::Mat<double> q_0(3,1); 
    q_0(0) = con.phi;
    q_0(1) = con.x; 
    q_0(2) = con.y;
    arma::Mat<double> m_0 = arma::zeros(2,1);
    
    
    for(int i=0; i<m_size; i++)
    {
        double r, phi_m; 
        r = measurement[i][0];
        phi_m = measurement[i][1];
        m_0(0) = (con.x + (r*cos(phi_m + double(con.phi)))); 
        m_0(1) = (con.y + (r*sin(phi_m + double(con.phi))));
        q_0 = arma::join_cols(q_0, m_0);
        
    }

    // q_0.print(std::cout << "q_0"); 


    arma::Mat<double> Sigma_0q = arma::zeros(3,3);
    arma::Mat<double> Sigma_0m = arma::eye(2*m_size,2*m_size);
    Sigma_0m = Sigma_0m*100000; 
    arma::Mat<double> zeros2n_3 = arma::zeros(2*m_size,3);
    arma::Mat<double> zeros3_2n = arma::zeros(3,2*m_size);
    arma::Mat<double> zeros2n_2n = arma::zeros(2*m_size, 2*m_size);

    arma::Mat<double> Sigma_0 = arma::join_rows(Sigma_0q, zeros3_2n);
    Sigma_0 = arma::join_cols(Sigma_0,arma::join_rows(zeros2n_3,Sigma_0m)); 
    ekf_slam.ekf_size(m_size);
    ekf_slam.initial_state(q_0, Sigma_0); 
    
    arma::Mat<double> Q = {{0.1, 0, 0},
                           {0, 0.1, 0}, 
                           {0, 0, 0.1}}; 
    
    arma::Mat<double> Q_bar = arma::join_cols(Q, zeros2n_3); 
    Q_bar = arma::join_rows(Q_bar, arma::join_cols(zeros3_2n,zeros2n_2n)); 
    
    arma::Mat<double> R = {{0.001, 0},
                           {0, 0.001}}; 
    ekf_slam.setQ(Q_bar); 
    ekf_slam.setR(R); 

    slam_flag = true; 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nuslam");
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
    nh.param("obs_radius",obs_radius,0.025);


    c.x = x0;
    c.y = yinit;
    c.phi = theta0;

    diff_drive = turtlelib::DiffDrive(dist, radius, phi, c);

    
    // subscribe to joint state
    joint_state_sub = nh.subscribe("joint_states", 1, joint_state_callback);
    // fake_sensor_sub = nh.subscribe("obstacle/fake_sensor", 5, fake_sensor_callback);
    fake_sensor_sub = nh.subscribe("landmarks", 5, fake_sensor_callback);
    
    ros::Timer slam_timer = nh.createTimer(ros::Duration(0.002), slam_timer_callback);

    // Assign the publisher odom
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    green_path_pub = nh.advertise<nav_msgs::Path>("/green_path", 10); 
    ros::Publisher blue_path_pub = nh.advertise<nav_msgs::Path>("/blue_path", 10); 
    green_sensor_pub = nh.advertise<visualization_msgs::MarkerArray>("green_sensor", 1, true);
    // Odom message
    odom.header.frame_id = "world";
    odom.child_frame_id = "blue_base_footprint";

    // Transform definition
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "blue_base_footprint";

    // map2odom
    map2odom_trans.header.frame_id = "map";
    map2odom_trans.child_frame_id = "odom"; 
    
    // odom2green
    odom2green_trans.header.frame_id = "odom"; 
    odom2green_trans.child_frame_id = "green_base_footprint"; 


    tf2_ros::TransformBroadcaster br;
    tf2_ros::TransformBroadcaster map_br; 
    tf2_ros::TransformBroadcaster green_br; 
    
    loop_rate.sleep();
    while(ros::ok())
    {
        if(measurement.size()>1)
        {
            if(slam_flag)
            {

            }
            else
            {
                initialize();
            }
            
        }
        if(slam_flag)
        {
            publish_slam();
            map_br.sendTransform(map2odom_trans);
            green_br.sendTransform(odom2green_trans); 
            blue_path_pub.publish(blue_path); 
        }
        
        publish_topics();
        odom_pub.publish(odom);
        br.sendTransform(transformStamped);
        
        ros::spinOnce();
        loop_rate.sleep();
        
        
    }

    return 0;

}


