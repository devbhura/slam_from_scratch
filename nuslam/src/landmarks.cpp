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
#include "tf2/LinearMath/Quaternion.h"

static ros::Publisher landmark_laser_pub, landmark_pub; 

void markers_landmarks(std::vector<std::vector<double>> circles)
{
    int num_of_circles = circles.size();
    visualization_msgs::MarkerArray landmarks_array; 
    landmarks_array.markers.resize(num_of_circles);

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

   landmark_pub.publish(landmarks_array);
}


// std::vector<std::vector<double>> circle_fit(std::vector<std::vector<turtlelib::Vector2D>> cluster_gp)
// {
//     int group_len = int(cluster_gp.size()); 

//     std::vector<std::vector<double>> circle_data; 

//     for(int i=0; i<group_len; i++)
//     {
//         // create variable for cluster
//         std::vector<turtlelib::Vector2D> cluster; 
//         cluster = cluster_gp.at(i); 
        
//         // variables for centroid
//         double c_x = 0.0; double c_y = 0.0; 

//         int len = cluster.size(); 
//         // Create vectors for x, y 
//         arma::Mat<double> X_arr(len,1), Y_arr(len,1), Z_arr(len,1); 
        
//         for(int j=0; j<len; j++)
//         {
//             turtlelib::Vector2D p; 
//             p = cluster.at(j); 

//             X_arr(j) = p.x; 
//             Y_arr(j) = p.y; 

//             c_x += p.x; 
//             c_y += p.y; 
//         }
        
//         // ROS_INFO_STREAM("X_arr before" << X_arr);
//         // ROS_INFO_STREAM("Y_arr before" << Y_arr); 
//         c_x = c_x/len; 
//         c_y = c_y/len; 

//         X_arr -= c_x; 
//         Y_arr -= c_y; 

//         // ROS_INFO_STREAM("X_arr after" <<  X_arr); 
//         // ROS_INFO_STREAM("Y_arr after" << Y_arr);
//         double z_mean = 0.0; 
//         for(int j=0; j<len; j++)
//         {
//             Z_arr(j) = pow(X_arr(j),2) + pow(Y_arr(j),2); 
//             z_mean += Z_arr(j); 
//         } 
//         z_mean = z_mean/len;

//         arma::Mat<double> Z = arma::join_rows(Z_arr,X_arr); 
        
//         Z = arma::join_rows(Z,Y_arr); 

//         arma::Mat<double> vec_ones = arma::ones(len,1); 

//         Z = arma::join_rows(Z,vec_ones); 

//         ROS_INFO_STREAM("Z after" <<  Z); 

//         arma::Mat<double> M = (Z.t())*(Z)/len; 

//         arma::Mat<double> H;
//         H = {{8*z_mean, 0, 0, 2},
//              {0,        1, 0, 0},
//              {0,        0, 1, 0},
//              {2,        0, 0, 0},
//             }; 
        
//         arma::Mat<double> H_inv; 
//         H_inv = arma::inv(H); 
//         // ROS_INFO_STREAM("H_inv" <<  H_inv);
//         arma::Mat<double> U, V;
//         arma::vec s, s_q; 
//         arma::svd(U,s,V,Z); 

         
//         double eig4 = s(3); 
//         // ROS_INFO_STREAM("V after" <<  V);
//         arma::Mat<double> A(4,1), Y_mat, Q_mat; 

//         if(eig4<10e-12)
//         {
//             A(0) = V(0,3); 
//             A(1) = V(1,3);
//             A(2) = V(2,3);
//             A(3) = V(3,3);
//         }
//         else{
//             arma::Mat<double> Sig, A_st(4,1), Eigvec; 
//             Sig = diagmat(s); 
//             // ROS_INFO_STREAM("Sigma" <<  Sig);
//             Y_mat = V*Sig*(V.t()); 
//             Q_mat = Y_mat*H_inv*Y_mat; 
//             // ROS_INFO_STREAM("Q" <<  Q_mat);
//             // ROS_INFO_STREAM("Y_mat" <<  Y_mat);
            
//             arma::eig_sym(s_q,Eigvec,Q_mat); 
//             int min_sig_index = 3;
//             // ROS_INFO_STREAM("s_q" <<  s_q);
//             for(int i=3; i>=0; i--)
//             {
//                 if(s_q(i)>0)
//                 {
//                     min_sig_index = i;
//                     break;
//                 } 
//             }
//             // ROS_INFO_STREAM("min_sig_index" <<  min_sig_index);

//             A_st(0) = Eigvec(0,min_sig_index); 
//             A_st(1) = Eigvec(1,min_sig_index); 
//             A_st(2) = Eigvec(2,min_sig_index); 
//             A_st(3) = Eigvec(3,min_sig_index);
//             A = inv(Y_mat)*A_st; 

//         }
//         // ROS_INFO_STREAM("A" <<  A);

//         double a = -A(1)/(2*A(0));  
//         double b = -A(2)/(2*A(0)); 
//         double R_s = (pow(A(1),2)+pow(A(2),2)-(4*A(0)*A(3)))/(4*pow(A(0),2)); 
//         a += c_x; 
//         b += c_y;
        
//         std::vector<double> data = {a,b,R_s}; 
//         ROS_INFO_STREAM("data a" <<  a);
//         ROS_INFO_STREAM("data b" <<  b);
//         ROS_INFO_STREAM("data R" <<  sqrt(R_s));
//         circle_data.push_back(data);         

//     }

//     return circle_data; 
// }


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

