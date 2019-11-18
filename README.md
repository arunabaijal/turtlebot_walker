[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# TurtleBot Walker
Roomba like behaviour

## Overview
This is a turtlebot application acting like roomba
Publisher - src/walker.cpp (velocities)
Subscriber - src/walker.cpp (laser input)

## Dependency
This module is built using ROS Kinetic and catkin.

ROS Kinetic installation steps - http://wiki.ros.org/kinetic/Installation/Ubuntu

## Clone and Build
Clone package into catkin workspace
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/arunabaijal/turtlebot_walker.git
  cd ~/catkin_ws
  source /opt/ros/kinetic/setup.bash
  catkin_make
  catkin_make install
  source ~/catkin_ws/devel/setup.bash
  ```

## Run
Open a new terminal and run the following commands for master
```
roscore
```

In another new terminal run the following commands for gazebo environment
  ```
  roslaunch turtlebot_gazebo turtlebot_world.launch
  ```

In a third new terminal run the following commands for running the roomba
  ```
  cd ~/catkin_ws/
  source ~/catkin_ws/devel/setup.bash 
  rosrun turtlebot_walker turtlebot_walker
  ```

## Run using launch file
After building the package
Open a new terminal and run the following commands for master
  ```
  roscore
  ```

Open a new terminal and run following command for running the roomba with gazebo
  ```
  roslaunch turtlebot_walker turtlebot_walker.launch StartRec:=true
  ```

To run without recording a bag file
  ```
  roslaunch turtlebot_walker turtlebot_walker.launch

# Playing bag files 
A recorded ros bag file is located in the results folder. To play the ros bag file type the following commands: 

In a new terminal 
```
roscore
```

Open another new terminal
```
cd ~/catkin_ws/src/turtlebot_walker/results
rosbag play turtlebot_walker.bag
```

You can view the nodes being published by running on a new terminal
```
rostopic echo /cmd_vel_mux/parameter_updates
```

