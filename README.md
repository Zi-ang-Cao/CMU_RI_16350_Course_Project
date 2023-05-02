# CMU 16350 Robot Planning Course_Project
* Title: Holonomic Robot Traversing a Mapped Environment with Dynamic Obstacles  
* Software: ROS, C++, Gazebo, Rviz  
* Author: Zi-ang Cao, Harvey Mei, Chris Suzuki  
* Date: Apr 30, 2023

# Table of Content
- [CMU 16350 Robot Planning Course\_Project](#cmu-16350-robot-planning-course_project)
- [Table of Content](#table-of-content)
  - [Installation](#installation) 
    - [Necessary Packages](#necessary-packages)
    - [Recommended Method of Installation](#recommended-method-of-installation) 
  - [Performance](#performance)
  - [How to execute the package](#how-to-execute-the-package)
    - [Terminal Command](#terminal-command)
      - [Specify "TURTLEBOT3\_MODEL" in launch file.](#specify-turtlebot3_model-in-launch-file)
      - [How to enable multiple Dynamic Obstacles](#how-to-enable-multiple-dynamic-obstacles)


# Installation
This package has been tested on Ubuntu 20.04 LTS with ROS [Noetic](http://wiki.ros.org/noetic). Make sure you have
installed the compatabile ROS version.
* [Install ROS1 Neotic on Ubuntu 20.04](https://wiki.ros.org/noetic/Installation/Ubuntu)

## Necessary Packages
1. Turtlebot3 Simulation - * [TurtleBot3 Installation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) 
2. Gazebo - * [Gazebo Installation](https://classic.gazebosim.org/tutorials?tut=ros_overview)

## Recommended Method of Installation
```Shell
git clone git@github.com:ice-bear-git/CMU_RI_16350_Course_Project.git
```

# Results


# How to execute the package
## Terminal Command
```Shell
cd ~/catkin_ws/src
source ../devel/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
roslaunch dynamic_obstacle_planner dynamic_obstacle_planner.launch
```

### Specify "TURTLEBOT3_MODEL" in launch file.
```Shell
cd ~/catkin_ws/src
source ../devel/setup.bash

roslaunch dynamic_obstacle_planner dynamic_obstacle_planner_with_waffle_pi.launch
```

### How to enable multiple Dynamic Obstacles
You might need to manually add topic in the Rviz to enable multiple dynamic obstacles. Here are the following instruction:
1. Click "add" in the left bottom corner
2. Select the interested topic to add.
3. Back to the left top menu bar to save the current configuration.
4. Once you save the configuration, those topic would be show up automatically.

