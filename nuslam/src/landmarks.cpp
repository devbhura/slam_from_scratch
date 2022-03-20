/// \file
/// \brief detects landmarks by clustering over laser scan and detecting circles
/**   
 * BROADCASTERS:
    * 
 * PUBLISHERS:
    *  landmark_laser_pub: publish cluster from laser
    *  landmark_pub: publish landmark based on circle fitting 
 * SERVICES:
    * 
 * SUBSCRIBERS:
    * 
 */

#include <ros/ros.h>
#include <armadillo>
#include <sensor_msgs/LaserScan.h>
#include "turtlelib/rigid2d.hpp"
#include "nuslam/ekf.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include "tf2/LinearMath/Quaternion.h"

static ros::Publisher landmark_laser_pub, landmark_pub; 

/// \brief
/// Publishes markers based on parameters from circle fitting
/// Input: 
/// \param circles - nested 2D vector that stores information about circle radius, x, and y
/// Output: Empty
void markers_landmarks(std::vector<std::vector<double>> circles)
{
    int num_of_circles = circles.size();
    visualization_msgs::MarkerArray landmarks_array; 
    landmarks_array.markers.resize(num_of_circles);
    if(num_of_circles>0){
    for (int i = 0; i<num_of_circles; i++)
    {
        std::vector<double> circle; 
        circle = circles.at(i); 
        landmarks_array.markers[i].header.frame_id = "red_base_footprint";
        landmarks_array.markers[i].header.stamp = ros::Time::now();
        landmarks_array.markers[i].id = i;

        landmarks_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
        landmarks_array.markers[i].action = visualization_msgs::Marker::ADD;

        landmarks_array.markers[i].pose.position.x = circle.at(0);
        landmarks_array.markers[i].pose.position.y = circle.at(1);
        landmarks_array.markers[i].pose.position.z = 0.0;

        tf2::Quaternion q_obs;
        q_obs.setRPY(0, 0,0);

        landmarks_array.markers[i].pose.orientation.x = q_obs.x();
        landmarks_array.markers[i].pose.orientation.y = q_obs.y();
        landmarks_array.markers[i].pose.orientation.z = q_obs.z();
        landmarks_array.markers[i].pose.orientation.w = q_obs.w();

        landmarks_array.markers[i].scale.x = 2*sqrt(circle.at(2));
        landmarks_array.markers[i].scale.y = 2*sqrt(circle.at(2));
        landmarks_array.markers[i].scale.z = 0.25;


        landmarks_array.markers[i].color.r = 1;
        landmarks_array.markers[i].color.g = 0;
        landmarks_array.markers[i].color.b = 1;
        landmarks_array.markers[i].color.a = 1;
        landmarks_array.markers[i].lifetime = ros::Duration(0.5); 

    }
    } 
   landmark_pub.publish(landmarks_array);
}

/// \brief
/// Converts from polar to cartesian
/// Input: 
/// \param r - range value from laser scan
/// \param i - the ith laser scan being computed
/// \param ang_inc - increment between two angles of laser scan 
/// Output:
/// \param vec - the cartesian x and y points 
turtlelib::Vector2D conv2cartesian(double r, int i, double ang_inc)
{
    turtlelib::Vector2D vec;
    double ang = ang_inc*i;
    ang = turtlelib::normalize_angle(ang); 
    // ROS_INFO_STREAM("Angle for laser" << ang); 
    vec.x = r*cos(ang); 
    vec.y = r*sin(ang); 

    return vec; 
    
}

/// \brief
/// laser scan callback. It clusters and fits a circle on the cluster 
/// Input: 
/// \param scan - laser scan
/// Output: Empty
void laser_callback(const sensor_msgs::LaserScan& scan)
{

    std::vector<std::vector<turtlelib::Vector2D>> cluster_group; 
    std::vector<turtlelib::Vector2D> cluster;
    double ang_inc = scan.angle_increment; 
    
    double dist_thres = 0.05; 

    int min_cluster_size = 4; 
    int max_cluster_size = 20; 
    
    int len = scan.ranges.size(); 
    int num_points = 0; 
    for(int i = 1; i<len; i++)
    {
        // clustering 
        double r = scan.ranges.at(i);
        double r_old = scan.ranges.at(i-1);
        turtlelib::Vector2D vec_r =  conv2cartesian(r, i+1, ang_inc); 
        turtlelib::Vector2D vec_r_old =  conv2cartesian(r_old, i, ang_inc); 
        if(cluster.size() == 0){
            cluster.push_back(vec_r_old); 
        }

        double dist = sqrt(pow(vec_r.x - vec_r_old.x, 2) + pow(vec_r.y - vec_r_old.y, 2)); 

        if(dist<dist_thres)
        {
            cluster.push_back(vec_r); 
        }
        else{
            if(cluster.size()>= min_cluster_size && cluster.size()<= max_cluster_size )
            {
                bool flag = slam::classify_circle(cluster); 
                if(flag == true)
                {
                cluster_group.push_back(cluster);
                num_points += cluster.size(); 
                }

            }
            cluster.clear(); 
            
        }


    }

    // circle fitting 
    if(cluster_group.size()>0)
    {
        std::vector<std::vector<double>> circles; 
        circles = slam::circle_fit(cluster_group); 
        markers_landmarks(circles); 
    }

    // Markers for clusters
    visualization_msgs::MarkerArray landmark_laser; 
    
    
    landmark_laser.markers.resize(num_points); 

    num_points = 0; 
    for(int j = 0; j<int(cluster_group.size()); j++)
    {
        cluster.clear(); 
        cluster = cluster_group.at(j); 

        for(int i = 0; i<int(cluster.size()); i++)
        {
            turtlelib::Vector2D p = cluster.at(i); 
            landmark_laser.markers[num_points].header.frame_id = "red_base_scan";
            landmark_laser.markers[num_points].header.stamp = ros::Time::now();
            landmark_laser.markers[num_points].id = num_points;

            landmark_laser.markers[num_points].type = visualization_msgs::Marker::SPHERE;
            landmark_laser.markers[num_points].action = visualization_msgs::Marker::ADD;

            landmark_laser.markers[num_points].pose.position.x = p.x;
            landmark_laser.markers[num_points].pose.position.y = p.y;
            landmark_laser.markers[num_points].pose.position.z = 0.0;
            landmark_laser.markers[num_points].scale.x = 0.01;
            landmark_laser.markers[num_points].scale.y = 0.01;
            landmark_laser.markers[num_points].scale.z = 0.01;


            landmark_laser.markers[num_points].color.r = 0;
            landmark_laser.markers[num_points].color.g = 1;
            landmark_laser.markers[num_points].color.b = 1;
            landmark_laser.markers[num_points].color.a = 1;
            landmark_laser.markers[num_points].lifetime = ros::Duration(1);
            num_points += 1; 
        }
    }

    landmark_laser_pub.publish(landmark_laser); 

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);
    

    ros::Subscriber laser_sub = nh.subscribe("scan", 10, laser_callback); 
    landmark_laser_pub = nh.advertise<visualization_msgs::MarkerArray>("landmark_cluster", 1);
    landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 1);
    
    while(ros::ok())
    {
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}

