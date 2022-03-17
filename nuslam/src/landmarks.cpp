/// \file
/// \brief 
/**
 * PARAMETERS:
    * timestep (double): tracks the current time step of the simulation
    
 * BROADCASTERS:
    * 
 * PUBLISHERS:
    *  
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

static ros::Publisher landmark_laser_pub; 

void circle_fit(std::vector<std::vector<turtlelib::Vector2D>> cluster_gp)
{
    for(int i=0; i<int(cluster_gp.size()); i++)
    {
        // create variable for cluster
        std::vector<turtlelib::Vector2D> cluster; 
        cluster = cluster_gp.at(i); 
        
        // variables for centroid
        double c_x = 0.0; double c_y = 0.0; 

        int len = cluster.size(); 
        // Create vectors for x, y 
        arma::Mat<double> X_arr(len,1), Y_arr(len,1), Z_arr(len,1); 
        
        for(int j=0; j<len; j++)
        {
            turtlelib::Vector2D p; 
            p = cluster.at(j); 

            X_arr(j) = p.x; 
            Y_arr(j) = p.y; 

            c_x += p.x; 
            c_y += p.y; 
        }
        
        // ROS_INFO_STREAM("X_arr before" << X_arr);
        // ROS_INFO_STREAM("Y_arr before" << Y_arr); 
        c_x = c_x/len; 
        c_y = c_y/len; 

        X_arr -= c_x; 
        Y_arr -= c_y; 

        // ROS_INFO_STREAM("X_arr after" <<  X_arr); 
        // ROS_INFO_STREAM("Y_arr after" << Y_arr);
        double z_mean = 0.0; 
        for(int j=0; j<len; j++)
        {
            Z_arr(j) = pow(X_arr(j),2) + pow(Y_arr(j),2); 
            z_mean += Z_arr(j); 
        } 
        z_mean = z_mean/len;

        arma::Mat<double> Z = arma::join_rows(Z_arr,X_arr); 
        
        Z = arma::join_rows(Z,Y_arr); 

        arma::Mat<double> vec_ones = arma::ones(len,1); 

        Z = arma::join_rows(Z,vec_ones); 

        ROS_INFO_STREAM("Z after" <<  Z); 


    }
}


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

void laser_callback(const sensor_msgs::LaserScan& scan)
{

    std::vector<std::vector<turtlelib::Vector2D>> cluster_group; 
    std::vector<turtlelib::Vector2D> cluster;
    double ang_inc = scan.angle_increment; 
    
    double dist_thres = 0.05; 

    int min_cluster_size = 3; 
    int max_cluster_size = 20; 
    
    int len = scan.ranges.size(); 
    int num_points = 0; 
    for(int i = 1; i<len; i++)
    {
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
                cluster_group.push_back(cluster);
                num_points += cluster.size(); 

            }
            cluster.clear(); 
            
        }


    }
    circle_fit(cluster_group); 
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

    
    while(ros::ok())
    {
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}

