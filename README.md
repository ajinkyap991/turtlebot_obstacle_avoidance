# turtlebot-obstacle-avoidance
[![License:MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/ajinkyap991/turtlebot_obstacle_avoidance/blob/Week12_HW/LICENSE)

Implementation of an obstacle avoidance algorithm for standard Turtlebot3 in Gazebo

## Author

Ajinkya Parwekar

## Overview

This ROS package implements a simple algorithm for the standard turtlebot3 to move around, successfully avoiding obstacles (similar to the Roomba vaccume cleaning robot). The algorithm logic is quite simple: The robot starts moving in the simulated world; The sensor attached to it detects for any obstacles ahead. If it comes upon any obstacle, it will change its direction by 80 degress and start moving forward again.

## License

Standard MIT License Clause

## Dependencies/Assumptions

- This package was created and tested on ROS Melodic on Ubuntu 18.04 and Gazebo 9.0.0 version.
- This package uses catkin for building the package.
- This packages follows C++ 11 standard of coding.
- This package needs standard turtlebot3 package installed on the system.

## Installation of standard turtlebot3 package:

The instructions to install the standard turtlebot3 ROS package can be found [here](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/).

## Steps to run the program
Open new terminal window and type the following:
```
cd << your catkin_ws/src >>
git clone https://github.com/ajinkyap991/turtlebot_obstacle_avoidance.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
roscore
```
Open a new terminal window and type:
```
roslaunch turtlebot_obstacle_avoidance obstacle_avoidance.launch
```

## Inspecting the Rosbag file
End the previous process, if its running and then type the following:
```
cd src/turtlebot_obstacle_avoidance/results/
rosbag info turtlebot_obstacle_avoidance.bag
```

## Playback of Rosbag file
Open new terminal window and type the following:
```
cd src/turtlebot_obstacle_avoidance/results/
rosbag play turtlebot_obstacle_avoidance.bag
```
## Cpplint check
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )

```

## Cppcheck check
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )

```