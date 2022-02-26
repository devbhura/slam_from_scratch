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
#include "sensor_msgs/LaserScan.h"
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
#include <random>
#include "ros/console.h"

static double timestep;
static double x, x0;
static double y, yinit; 
static double theta, theta0;
static double obs_radius, x_length, y_length;
static visualization_msgs::MarkerArray obstacles_array, wall_array, fake_sensor;
static std::vector<double> obstacles_x_arr, obstacles_y_arr, obstacles_theta_arr, wall_x_arr, wall_y_arr;
static ros::Publisher obstacle_marker, wall_marker, snsr_data, fake_sensor_pub, lidar_pub;
static turtlelib::DiffDrive diff_drive;
static double motor_cmd_to_radsec;
static double encoder_ticks_to_rad;
static turtlelib::WheelPhi old_phi;
static int r;
static double dist, radius;
static turtlelib::Config q, q_old;
static nuturtlebot_msgs::SensorData sensor;
static double noise_mean, noise_stdev;
static double slip_min, slip_max;
static std::normal_distribution<double> noise;
static std::uniform_real_distribution<double> slip;
static std::default_random_engine generator;
static double basic_sensor_variance, max_range, collision_radius;
static bool fake_sensor_flag = false;
static bool lidar_flag = false;
static sensor_msgs::LaserScan scan;
static double scan_angle_max, scan_angle_min, samples, scan_range_min, scan_range_max;

bool checkCollision(turtlelib::Config q);
int sgn(double v);
std::mt19937 & get_random()
{
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    return mt;
}

int sgn(double v)
{
    if (v < 0.0) return -1;
    if (v > 0.0) return 1;
    return 0;
}

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

///\brief
/// timer callback
/// Input: None
/// Output: None
void timer_callback(const ros::TimerEvent&)
{

    fake_sensor.markers.resize(obstacles_x_arr.size());
    turtlelib::Config robot_state = diff_drive.getConfig();
    std::normal_distribution<double> fake_sensor_noise(0.0,basic_sensor_variance);
    turtlelib::Vector2D vec = {robot_state.x, robot_state.y};
    turtlelib::Transform2D Twb(vec, robot_state.phi);


    for (int i = 0; i<obstacles_x_arr.size(); i++)
    {
        double distance = sqrt(pow(robot_state.x - obstacles_x_arr[i], 2) + pow(robot_state.y - obstacles_y_arr[i], 2));
        turtlelib::Vector2D obs_vec = {obstacles_x_arr[i], obstacles_y_arr[i]};
        turtlelib::Transform2D Two(obs_vec), Tbo;
        Tbo = (Twb.inv())*Two;
        turtlelib::Vector2D body_vec = Tbo.translation();


        // ROS_INFO_STREAM("distance: %f" << distance);
        
        fake_sensor.markers[i].header.frame_id = "red_base_footprint";
        fake_sensor.markers[i].header.stamp = ros::Time::now();
        fake_sensor.markers[i].id = i;

        fake_sensor.markers[i].type = visualization_msgs::Marker::CYLINDER;
        fake_sensor.markers[i].action = visualization_msgs::Marker::ADD;

        if(distance > max_range)
        {
            fake_sensor.markers[i].action = visualization_msgs::Marker::DELETE;
            // ROS_INFO_STREAM("DEL added");
        }
        
        double sennoi = fake_sensor_noise(get_random());
        // ROS_INFO_STREAM("Noise: %f" << sennoi );
        // ROS_INFO_STREAM("Var: %f" << basic_sensor_variance );

        double r = sqrt(pow(body_vec.x,2) + pow(body_vec.y,2));
        double ang = atan2(body_vec.y,body_vec.x);

        r += sennoi;
        ang += sennoi;

        fake_sensor.markers[i].pose.position.x = r*cos(ang);
        fake_sensor.markers[i].pose.position.y = r*sin(ang);
        fake_sensor.markers[i].pose.position.z = 0.0;

        tf2::Quaternion q_obs;
        q_obs.setRPY(0, 0, obstacles_theta_arr[i]);

        fake_sensor.markers[i].pose.orientation.x = q_obs.x();
        fake_sensor.markers[i].pose.orientation.y = q_obs.y();
        fake_sensor.markers[i].pose.orientation.z = q_obs.z();
        fake_sensor.markers[i].pose.orientation.w = q_obs.w();

        fake_sensor.markers[i].scale.x = 2*obs_radius;
        fake_sensor.markers[i].scale.y = 2*obs_radius;
        fake_sensor.markers[i].scale.z = 0.25;


        fake_sensor.markers[i].color.r = 1;
        fake_sensor.markers[i].color.g = 1;
        fake_sensor.markers[i].color.b = 0;
        fake_sensor.markers[i].color.a = 1;

    }

    fake_sensor_flag = true;

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


    wall_array.markers[0].color.r = 1;
    wall_array.markers[0].color.g = 0;
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


    wall_array.markers[1].color.r = 1;
    wall_array.markers[1].color.g = 0;
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


    wall_array.markers[2].color.r = 1;
    wall_array.markers[2].color.g = 0;
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


    wall_array.markers[3].color.r = 1;
    wall_array.markers[3].color.g = 0;
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

    double wheel_noise = noise(get_random());
    double eta = slip(get_random());

    u.x = input.left_velocity;
    u.y = input.right_velocity;

    if(u.x!=0){ u.x += wheel_noise;}
    if(u.y!=0){ u.y += wheel_noise;}

    u.x = u.x*motor_cmd_to_radsec;
    u.y = u.y*motor_cmd_to_radsec;

    // Multiply by slip 
    u.x = eta*u.x;
    u.y = eta*u.y;

    old_phi.left_phi += u.x/r;
    old_phi.right_phi += u.y/r;



    q = diff_drive.ForwardKin(old_phi);

    bool state = checkCollision(q);
    if(state)
    {
       q = q_old;
    }
    else
    {
        q_old = q;
    }

    diff_drive = turtlelib::DiffDrive(dist, radius, old_phi, q);

    sensor.left_encoder = int(old_phi.left_phi/encoder_ticks_to_rad);
    sensor.right_encoder = int(old_phi.right_phi/encoder_ticks_to_rad);

    
}

bool checkCollision(turtlelib::Config q)
{
    double coll_dist;
    bool state = false;
    for (int i = 0; i<obstacles_x_arr.size(); i++)
    {
        coll_dist = sqrt(pow(q.x - obstacles_x_arr[i],2) + pow(q.y - obstacles_y_arr[i],2));
        if(turtlelib::almost_equal(coll_dist,(collision_radius+obs_radius), 0.01))
        {state = true;}
    }

    return state;
}

void lidar_timer_callback(const ros::TimerEvent&)
{

    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "red_base_scan";
    scan.angle_min = scan_angle_min;
    scan.angle_max = scan_angle_max;
    scan.angle_increment = 6.28/samples;
    scan.time_increment = 0.2/samples;
    scan.range_min = scan_range_min;
    scan.range_max = scan_range_max;
    scan.ranges.resize(samples);

    double ang = 0.0;
    turtlelib::Config robot_state = diff_drive.getConfig();
    turtlelib::Vector2D vec = {robot_state.x, robot_state.y};
    turtlelib::Transform2D Twr(vec, robot_state.phi), Trw;
    Trw = Twr.inv();

    for (int j = 0; j<samples; j++)
    {
        double x1, y1, x2, y2, D, dr;
        x1 = 0.2*cos(ang);
        y1 = 0.2*sin(ang);
        x2 = scan_range_max*cos(ang);
        y2 = scan_range_max*sin(ang);
        turtlelib::Vector2D v1{x1, y1}, v2{x2, y2}, v1_obs, v2_obs;
        std::vector<double> range; 
        range.push_back(scan_range_max - 0.1);
        for (int i = 0; i<obstacles_x_arr.size(); i++)
        {
            turtlelib::Vector2D obs_vec{obstacles_x_arr[i], obstacles_y_arr[i]};
            turtlelib::Transform2D Two(obs_vec), Tor, Tro;
            Tor = (Two.inv())*Twr;
            Tro = Tor.inv();
            v1_obs = Tor(v1);
            v2_obs = Tor(v2);

            dr = sqrt(pow(v2_obs.x - v1_obs.x,2)+pow(v2_obs.y - v1_obs.y,2));
            D = v1_obs.x*v2_obs.y - v2_obs.x*v1_obs.y;

            double delt = sqrt(pow(obs_radius,2)*pow(dr,2) - pow(D, 2));
            // ROS_INFO_STREAM("delt: %f" << delt );
            
            if(delt >= 0.0)
            {
                double x, y, dx, dy;
                dy = v2_obs.y - v1_obs.y;
                dx = v2_obs.x - v1_obs.x;
                x = (D*(dy) - sgn(dy)*dx*delt)/pow(dr,2);
                y = (-D*(dx) - abs(dy)*delt)/pow(dr,2);
                turtlelib::Vector2D inter{x,y}, inter_r;
                inter_r = Tro(inter);
                double inter_dist = sqrt(pow(inter_r.x,2) + pow(inter_r.y,2));
                if(sqrt(pow(inter_r.x-v2_obs.x,2) + pow(inter_r.y-v2_obs.y,2)) <inter_dist){
                    range.push_back(inter_dist);
                }
                x = (D*(dy) + sgn(dy)*dx*delt)/pow(dr,2);
                y = (-D*(dx) + abs(dy)*delt)/pow(dr,2);
                turtlelib::Vector2D inter_n{x,y};
                inter_r = Tro(inter_n);
                inter_dist = sqrt(pow(inter_r.x,2) + pow(inter_r.y,2));
                if(sqrt(pow(inter_r.x-v2_obs.x,2) + pow(inter_r.y-v2_obs.y,2)) <inter_dist){
                    range.push_back(inter_dist);
                }
            }

        }

        for(int k=0; k<(wall_x_arr.size()-1); k++)
        {
            turtlelib::Vector2D line_p3{wall_x_arr[k], wall_y_arr[k]};
            turtlelib::Vector2D line_p4{wall_x_arr[k+1], wall_y_arr[k+1]};
            turtlelib::Vector2D line_p3_r, line_p4_r;
            line_p3_r = Trw(line_p3);
            line_p4_r = Trw(line_p4);
            double x3, y3, x4, y4;
            x3 = line_p3.x;
            x4 = line_p4.x;
            y3 = line_p3.y;
            y4 = line_p4.y;

            turtlelib::Vector2D v1_w, v2_w;
            v1_w = Twr(v1);
            v2_w = Twr(v2);
            x1 = v1_w.x;
            x2 = v2_w.x;
            y1 = v1_w.y;
            y2 = v2_w.y;

            double Px, Py, D, r_d;
            D = ((x1 - x2)*(y3 - y4)) - ((y1-y2)*(x3 - x4));
            Px = ((((x1*y2) - (y1*x2))*(x3-x4)) - ((x1-x2)*((x3*y4) - (y3*x4))))/D;
            Py = ((((x1*y2) - (y1*x2))*(y3-y4)) - ((y1-y2)*((x3*y4) - (y3*x4))))/D;

            turtlelib::Vector2D P_r, P_w{Px,Py};
            // ROS_INFO_STREAM("P_w: %f" << P_w );
            P_r = Trw(P_w);
            // ROS_INFO_STREAM("P_r: %f" << P_r );
            r_d = sqrt(pow(P_r.x,2)+pow(P_r.y,2));
            // ROS_INFO_STREAM("r_d: %f" << r_d );
            if(sqrt(pow(P_r.x - scan_range_max*cos(ang),2)+pow(P_r.y - scan_range_max*sin(ang),2)) <r_d)
            {
                range.push_back(abs(r_d));
            }
            

        }
        
        scan.ranges[j] = *min_element(range.begin(), range.end());

        ang += 6.28/samples;
        // ROS_INFO_STREAM("Ang: %f" << ang );
        if(ang >= 6.28)
        {
            ang = 0.0;
        }
    }

    lidar_flag = true;

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
    ros::Timer timer_sensor = nh.createTimer(ros::Duration(0.2), timer_callback);
    ros::Timer timer_lidar = nh.createTimer(ros::Duration(0.2), lidar_timer_callback);
    // ros::Publisher joint_msg_pub = red.advertise<sensor_msgs::JointState>("joint_states", 1);
    obstacle_marker = obstacle.advertise<visualization_msgs::MarkerArray>("obs", 1, true);
    wall_marker = obstacle.advertise<visualization_msgs::MarkerArray>("wall", 1, true);
    snsr_data = red.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 10);
    fake_sensor_pub = obstacle.advertise<visualization_msgs::MarkerArray>("fake_sensor", 1, true);
    lidar_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10, true);

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
    nh.param("noise_mean", noise_mean, 0.0);
    nh.param("noise_stdev", noise_stdev, 1.0);
    nh.param("slip_max", slip_max,1.0);
    nh.param("slip_min", slip_min,1.0);
    nh.param("basic_sensor_variance", basic_sensor_variance, 0.1);
    nh.param("max_range", max_range, 2.0);
    nh.param("collision_radius", collision_radius, 0.11);
    nh.param("scan_angle_min", scan_angle_min, 0.0);
    nh.param("scan_angle_max", scan_angle_max, 6.28319);
    nh.param("scan_range_min", scan_range_min, 0.12);
    nh.param("scan_range_max", scan_range_max, 3.5);
    nh.param("samples", samples, 360.0);

    // noise and slip
    noise.param( decltype(noise)::param_type{ noise_mean, noise_stdev } ) ;
    slip.param( decltype(slip)::param_type{slip_min, slip_max}); 

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

    // Walls marker position arrays
    wall_x_arr = {x_length/2, -x_length/2, -x_length/2, x_length/2, x_length/2};
    wall_y_arr = {y_length/2, y_length/2, -y_length/2, -y_length/2, y_length/2};
    
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

        if(fake_sensor_flag)
        {
            fake_sensor_pub.publish(fake_sensor);
            fake_sensor_flag = false;
        }

        if(lidar_flag)
        {
            lidar_pub.publish(scan);
            lidar_flag = false;
        }
        
        snsr_data.publish(sensor);
        ros::spinOnce();

        loop_rate.sleep();
        
    }

    return 0;

}