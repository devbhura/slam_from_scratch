# nuslam package

## Description
This package implements an Extended Kalman Filter that measures landmarks from scanner and estimates the state of the robot. 

### Demonstration of slam in simulation

![image](./images/slam_3robots.png)

The blue robot(representing just odometry) diverges due to slip. The green robot representing the EKF state estimate is close to the actual robot (represented by red). Since noise is enabled, green robot does not follow the red robot perfectly but it stays very close to it. 


## Launchfiles 

To launch in simulation: run `roslaunch nuslam slam.launch`

By default, it loads up the circle node and allows you to drive the robot in a circle. To start the teleop node, one can run `roslaunch nuslam slam.launch cmd_src:=teleop`