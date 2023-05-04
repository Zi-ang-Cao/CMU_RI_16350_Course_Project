# CMU 16350 Robot Planning Course_Project
* Title: Holonomic Robot Traversing a Mapped Environment with Dynamic Obstacles  
* Software: ROS, C++, Gazebo, Rviz  
* Author: Zi-ang Cao, Harvey Mei, Chris Suzuki  
* Date: Apr 30, 2023

# Table of Content
- [CMU 16350 Robot Planning Course\_Project](#cmu-16350-robot-planning-course_project)
- [Table of Content](#table-of-content)
  - [Environment](#environment)
    - [Necessary Packages](#necessary-packages)
    - [Supporting Config Instruction](#supporting-config-instruction)
  - [Result](#result)
  - [How to execute the package](#how-to-execute-the-package)
    - [Basic](#basic)
      - [Single Dynamic Obstacles](#single-dynamic-obstacles)
      - [multiple Dynamic Obstacles](#multiple-dynamic-obstacles)
    - [Advanced](#advanced)
      - [Specify "TURTLEBOT3\_MODEL" in launch file.](#specify-turtlebot3_model-in-launch-file)
      - [Run with multiple Ros workspace.](#run-with-multiple-ros-workspace)


## Environment
This package has been tested on Ubuntu 20.04 LTS with ROS [Noetic](http://wiki.ros.org/noetic). Make sure you have installed the compatabile ROS version. 

```Shell
git clone https://github.com/Zi-ang-Cao/CMU_RI_16350_Course_Project.git
```

### Necessary Packages
* Ubuntu 20.04 LTS
* ROS [Noetic](http://wiki.ros.org/noetic)
* Turtlebot3 Simulation -- [TurtleBot3 Installation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) 
* Gazebo -- [Gazebo Installation](https://classic.gazebosim.org/tutorials?tut=ros_overview)
* gazebo_ros_pkgs -- [gazebo_ros_pkgs Installation](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

### Supporting Config Instruction
* [GitHub Command Line]()
* [ROS Relevant Setup]()
* [How to debug ROS with VSCode](https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md)


## Result
Red lines represent the replanning efforts. Meanwhile, We have both ideal pseudo Robot agent and more realistic PIT agent.  

* Single Dynamic Obstacles  
  <img src="/Support/result.png" title="Partial Re-Planning">

* Multiple Dynamic Obstacles
  - Total AStar -- [youtube](https://www.youtube.com/watch?v=cTupE73wB80)
  - Partial Astar -- [youtube](https://www.youtube.com/watch?v=e3G6Rci00EY)
    + Simplified Incremental Search


## How to execute the package
### Basic
#### Single Dynamic Obstacles
```Shell
cd ~/catkin_ws/src
source ../devel/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
roslaunch dynamic_obstacle_planner dynamic_obstacle_planner.launch
```

#### multiple Dynamic Obstacles
You might need to manually add topic in the Rviz to enable multiple dynamic obstacles. Here are the following instruction:
1. Click "add" in the left bottom corner
2. Select the interested topic to add.
3. Back to the left top menu bar to save the current configuration.
4. Once you save the configuration, those topic would be show up automatically.


### Advanced

#### Specify "TURTLEBOT3_MODEL" in launch file.
1. Add the following line in the launch.md file.
```Shell
<env name="TURTLEBOT3_MODEL" value="waffle_pi"/>
```

2. Run the following command
```Shell
cd ~/catkin_ws/src
source ../devel/setup.bash

roslaunch dynamic_obstacle_planner dynamic_obstacle_planner_with_waffle_pi.launch
```

#### Run with multiple Ros workspace.
1. Source the interested workspace
```Shell
""" Either update the command in bashrc """
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# echo "source ~/catkin_ws_CMU_16350/devel/setup.bash" >> ~/.bashrc

""" Or specify corresponding Ros workspace in each new termial windown """
cd ~/catkin_ws_CMU_16350/src
source ../devel/setup.bash
```

2. Check the current ROS_Package_Path here
```Shell
echo $ROS_PACKAGE_PATH
```