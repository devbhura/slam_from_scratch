#include "catch_ros/catch.hpp"
#include "nuslam/ekf.hpp"
#include "turtlelib/rigid2d.hpp"
#include <sstream>
#include <cmath> 
#include <armadillo>
#include <iostream>
#include <ros/console.h>

double e = 0.001; 

TEST_CASE("Circle Test", "[Cirlcle fit]") { //Devesh Bhura
    // turtlelib::Vector2D p;

    arma::Mat<double> l = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}}; 

    std::vector<std::vector<turtlelib::Vector2D>> cluster_group; 
    std::vector<turtlelib::Vector2D> cluster;
    int len = 6; 
    // std::cout << "len " << len << std::endl; 

    for(int i = 0; i<len; i++)
    {
        // std::cout << "i " << i << std::endl;
        turtlelib::Vector2D p;
        p.x = l(i,0); 
        p.y = l(i,1); 

        // std::cout << p.x << " y " << p.y << std::endl;
        cluster.push_back(p); 
    }
    cluster_group.push_back(cluster); 

    

    std::vector<std::vector<double>> circles;
    circles = slam::circle_fit(cluster_group);

    // std::cout << circles << std::endl; 
    
    // ROS_INFO_STREAM("Circles"<< circles.size()); 
    std::vector<double> circle; 
    std::cout << " circle_size " << circle.size() << std::endl;
    circle = circles.at(0); 
    double x_check = 4.615482;
    double y_check = 2.807354;
    double radius = 4.8275;
    double c_r = (sqrt(circle.at(2))); 
    CHECK(x_check == Approx(circle.at(0))); 
    CHECK(y_check == Approx(circle.at(1)));
    CHECK(radius == Approx(c_r).margin(e)); 
}

TEST_CASE("Circle Test 2", "[Circle fit 2]") { //Devesh Bhura
    // turtlelib::Vector2D p;

    arma::Mat<double> l = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}}; 

    std::vector<std::vector<turtlelib::Vector2D>> cluster_group; 
    std::vector<turtlelib::Vector2D> cluster;
    int len = 4; 
   
    for(int i = 0; i<len; i++)
    {
        turtlelib::Vector2D p;
        p.x = l(i,0); 
        p.y = l(i,1); 
        cluster.push_back(p); 
    }
    cluster_group.push_back(cluster); 
    std::vector<std::vector<double>> circles;
    circles = slam::circle_fit(cluster_group);
    std::vector<double> circle; 
    circle = circles.at(0); 
    double x_check = 0.4908357;
    double y_check =  -22.15212;
    double radius = 22.17979;
    double c_r = (sqrt(circle.at(2))); 
    CHECK(x_check == Approx(circle.at(0)).margin(e)); 
    CHECK(y_check == Approx(circle.at(1)).margin(e));
    CHECK(radius == Approx(c_r).margin(e)); 
}