# CMU 16350 Robot Planning Course_Project
* Title: Holonomic Robot Traversing a Mapped Environment with Dynamic Obstacles  
* Software: ROS, C++, Gazebo, Rviz  
* Author: Zi-ang Cao, Harvey Mei, Chris Suzuki  
* Date: Apr 30, 2023

# Table of Content
- [CMU 16350 Robot Planning Course\_Project](#cmu-16350-robot-planning-course_project)
- [Table of Content](#table-of-content)
  - [Performance](#performance)
  - [How to execute the code](#how-to-execute-the-code)
    - [Terminal Command](#terminal-command)
      - [Specify "TURTLEBOT3\_MODEL" in launch file.](#specify-turtlebot3_model-in-launch-file)
      - [Run with multiple Ros workspace.](#run-with-multiple-ros-workspace)
    - [How to enable multiple Dynamic Obstacles](#how-to-enable-multiple-dynamic-obstacles)
  - [Install Instruction](#install-instruction)
    - [ROS Neotic](#ros-neotic)
      - [Create a ROS Workspace](#create-a-ros-workspace)
      - [Create ROS Package](#create-ros-package)
      - [Create Additional Catkin WorkSpace](#create-additional-catkin-workspace)
    - [ROS dependence](#ros-dependence)
      - [Gazebo](#gazebo)
        - [gazebo\_ros\_pkgs (ROS Neotic)](#gazebo_ros_pkgs-ros-neotic)
        - [Testing Gazebo with ROS Integration](#testing-gazebo-with-ros-integration)
      - [TurtleBot3](#turtlebot3)
  - [GitHub Commend Line](#github-commend-line)
    - [Setup Git SSH](#setup-git-ssh)
    - [Change/Create Git Branch](#changecreate-git-branch)
    - [Enforce Git Pull](#enforce-git-pull)
    - [Enforce Git Push](#enforce-git-push)
    - [Git Basic](#git-basic)


## Performance

## How to execute the code
### Terminal Command
```Shell
cd ~/catkin_ws/src
source ../devel/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
roslaunch dynamic_obstacle_planner dynamic_obstacle_planner.launch
```

#### Specify "TURTLEBOT3_MODEL" in launch file.
```Shell
cd ~/catkin_ws/src
source ../devel/setup.bash

roslaunch dynamic_obstacle_planner dynamic_obstacle_planner_with_waffle_pi.launch
```

#### Run with multiple Ros workspace.
```Shell
""" Either update the command in bashrc """
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# echo "source ~/catkin_ws_CMU_16350/devel/setup.bash" >> ~/.bashrc

""" Or specify corresponding Ros workspace in each new termial windown """

cd ~/catkin_ws_CMU_16350/src
source ../devel/setup.bash

""" Check the current ROS_Package_Path here """
echo $ROS_PACKAGE_PATH

```

### How to enable multiple Dynamic Obstacles
You might need to manually add topic in the Rviz to enable multiple dynamic obstacles. Here are the following instruction:
1. Click "add" in the left bottom corner
2. Select the interested topic to add.
3. Back to the left top menu bar to save the current configuration.
4. Once you save the configuration, those topic would be show up automatically.




## Install Instruction

### ROS Neotic
* [Install ROS1 Neotic on Ubuntu 20.04](https://wiki.ros.org/noetic/Installation/Ubuntu)
```Shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full

apt search ros-noetic

source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
#### Create a ROS Workspace
```Shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash
echo $ROS_PACKAGE_PATH
```

#### Create ROS Package
* Recommend way to create a Tutorial
```Shell
cd ~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

cd ~/catkin_ws
catkin_make

. ~/catkin_ws/devel/setup.bash

rospack depends1 beginner_tutorials 
```

#### Create Additional Catkin WorkSpace
```Shell
source /opt/ros/noetic/setup.bash
# Must creat the "src" folder inside of the workspace before runing "catkin_make"
mkdir -p ~/catkin_ws_CMU_16350/src

git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/catkin_ws_CMU_16350/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
```

-----------------

### ROS dependence

#### Gazebo
```Shell
curl -sSL http://get.gazebosim.org | sh

gazebo --version
which gzserver
which gzclient
```

##### gazebo_ros_pkgs (ROS Neotic)
```Shell
# A. Install Pre-Built Debian\Ubuntu binary packages
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Clone the Github Repositories
sudo apt-get install git
sudo apt-get install -y libgazebo11-dev
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel

rosdep install --from-paths . --ignore-src --rosdistro noetic -y

cd ~/catkin_ws/
catkin_make
```



#### TurtleBot3
* [TurtleBot3 Installation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) 


## GitHub Command Instruction

### Setup Git SSH
* [Guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux)

# Recommended Method of Installation
git clone git@github.com:ice-bear-git/CMU_RI_16350_Course_Project.git -b ZiangCao

