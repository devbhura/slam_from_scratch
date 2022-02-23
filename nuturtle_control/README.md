# nuturtle_control package

## Description 
This package controls the motion of the robot. The user can move the robot using teleop_key or move it in a circle

## Launchfiles

To launch simulation, run `roslaunch nuturtle_control start_robot.launch`

By default, it loads up the circle node and allows you to drive the robot in a circle. To start the teleop node, one can run `roslaunch nuturtle_control start_robot.launch cmd_src:=teleop`

To launch on the actual robot, use `roslaunch nuturtle_control basic_remote.launch`

## Physical experiment

### Driving in a Straight Line: 

The robot was driven several times in a straight line. It started at the position (0.18, 0.022) and ended at (0.2, 0.027). 

https://user-images.githubusercontent.com/55405657/153738399-6d8f5008-b316-4df7-9455-945f0b55ad4d.mp4

![straight_line](https://user-images.githubusercontent.com/55405657/153738907-3ee43f76-e0f1-43f2-8c78-0a1e94e365ea.gif)

### Rotation

The robot was rotated clockwise and anti-clockwise several times. It started at the position  (0.126, -0.025) with an orientaiton (0, 0, -0.19, 0.98) and ends at position (0.25, -0.028) with an orientation of (0, 0, 0.98, 0.17). 

https://user-images.githubusercontent.com/55405657/153738404-2ad4104f-0ca8-4dbb-a7c0-f16b32d4cf13.mp4

![rotation](https://user-images.githubusercontent.com/55405657/153738911-637b8404-8437-48ed-b323-5bee0b44ca29.gif)


### Driving in a Circle

The robot was driven in a circle several times. It started at the position (0.018, 0.105) with an orientaiton (0,0,0.98,0.17) and ends at position (-0.034,-0.27) with an orientation of (0,0,-0.21, -0.98). 

https://user-images.githubusercontent.com/55405657/153738394-891266cb-610d-480c-b441-862710f66bea.mp4

![circle](https://user-images.githubusercontent.com/55405657/153738915-655cadf4-6238-4c1a-94c9-334b34dbce91.gif)


### Worst Case in a stright line

The robot was driven again several times in a straight line, but faster. It started at the position (0.15, -0.014) and orientation in quaternion (0,0,-0.09, 0.99) and ended at (0.2, 0.06) with an orientation (0,0,0.0023,0.99). Making it faster made the odometry estimate worse. 

https://user-images.githubusercontent.com/55405657/153738391-1ca5df5a-9360-4cac-9a67-8d4430027961.mp4

![best_worst](https://user-images.githubusercontent.com/55405657/153738922-fc8e641d-dc9f-4f2e-a4a2-cef80d2d6684.gif)

* All values mentioned are as per calculated by odometry