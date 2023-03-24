# Instruction of Workflow Setup on Mac

## M-Chip Mac + Parallel Ubuntu + [RoboStack + Gazebo + TurtleBot3] -- Installation

### Parallel Virtual Machine

#### Parallel Desktop & Ubuntu
* Setup [Parallel Desktop](https://www.parallels.com/products/desktop/welcome-trial/) with activation key (I got one from UPitt student software service)
* Create an `Ubuntu 22.04` instance and **must install the `Parallel ToolBox` inside of Ubuntu** for easy **copy and paste**. Refer to this [link](https://download.parallels.com/desktop/v12/docs/en_US/Parallels%20Desktop%20User%27s%20Guide/32925.htm)
	- ***Note***: I can use `Command+C` to copy everywhere except of the Ubuntu terminal. In the Ubuntu terminal, I need to press `Shift + Control + V` to paste the pre-copied stuff.

#### Useful Software on Ubuntu
* Install **[Sublime Text]** on [Ubuntu with apt](https://www.sublimetext.com/docs/linux_repositories.html#apt)
	- Press `Command + Shift + P` to install packages controller and install useful Packages:
		+ `MarkdownPreview`, `MarkdownEditing`
	- To avoid `github_oauth_token's rate limit` issue from the begining!
		+ Open `Preferences.sublime-settings` file: Preference>>Settings
		+ Add the following line `"enable_autoreload": false` into the large curly brace 
```Shell
# Install the GPG key:
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/sublimehq-archive.gpg > /dev/null

# Select the channel [Stable] to use:
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list

# Update apt sources and install Sublime Text:
sudo apt-get update
sudo apt-get install sublime-text
```

* Install **[VSCode]** on [Arm64 Ubuntu](https://linuxiac.com/install-visual-studio-code-on-ubuntu-22-04/)
	- Install Useful Extension: 
		+ `XMake`, `codelldb`, `Python`, `C`, `Remote - SSH`, `Rainbow CSV`, `Lua`
```Shell
# 1. Install Prerequisites
sudo apt install software-properties-common apt-transport-https wget gpg

# 2. Import Microsoft’s GPG Key
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg

# 3. Add Microsoft’s Visual Studio Code Repository
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'

# 4. Run System Update
sudo apt update

# 5. Install VS Code on Ubuntu 22.04
sudo apt install code
```


### ROS on (M-Chip) Mac
#### Prepare [Miniconda](https://docs.conda.io/en/latest/miniconda.html#linux-installers)
> Setting up a virtual environment along the way would be soooooo-helpful!
```Shell
# curl -O <Url_To_Download>
curl -O https://repo.anaconda.com/miniconda/Miniconda3-py310_23.1.0-1-Linux-aarch64.sh

# Include the bash command regardless of whether or not you are using a Bash shell.
bash https://repo.anaconda.com/miniconda/Miniconda3-py310_23.1.0-1-Linux-aarch64.sh
```

#### Install [ROS-Noetic] using [RoboStack](https://robostack.github.io/index.html)
> Ros-Noetic is somehow the most compatible version
```Shell
# if you don't have mamba yet, install it first (not needed when using mambaforge):
conda install mamba -c conda-forge

# now create a new environment [For ROS1 Noetic and ROS2 Galactic]
mamba create -n ros_env python=3.9 -c conda-forge

# In a new terminla 
conda activate ros_env

# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults

# Install the version of ROS you are interested in:
mamba install ros-noetic-desktop  

# optionally, install some compiler packages if you want to e.g. build packages in a colcon_ws:
mamba install compilers cmake pkg-config make ninja colcon-common-extensions

# on Linux and osx (but not Windows) for ROS1 you might want to:
mamba install catkin_tools

# note that in this case, you should also install the necessary dependencies with conda/mamba, if possible

# reload environment to activate required scripts before running anything
# on Windows, please restart the Anaconda Prompt / Command Prompt!
conda deactivate
conda activate ros_env

# if you want to use rosdep, also do:
mamba install rosdep
rosdep init  # note: do not use sudo!
rosdep update
```

* ***Notes***: From now, being expected of geting fail by using defualt installation syntax as `sudo apt-get`. On Arm64 Ubuntu, many things become different!!! ***[Try to using `mamba` for the most of time!!!]()***
	- `mamba is a reimplementation of the conda package manager in C++.`

#### Setup Gazebo_Ros
1. [Gazebo using mamba/miniconda](https://github.com/conda-forge/gazebo-feedstock)
> Notes: Always using `mamba` to install dependencies! `sudo apt-get` is so easy to fail on Arm64 linux.
```Shell
mamba install gazebo
gazebo
```

2. []()
```Shell
mamba search ros-kinetic-gazebo-ros --channel conda-forge
```
3. []()
```Shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

conda

```
#### Setup [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
> Notes: Always using `mamba` to install dependencies! `sudo apt-get` is so easy to fail on Arm64 linux.
```Shell

https://anaconda.org/robostack/ros-noetic-gazebo-ros-control

https://anaconda.org/robostack/ros-noetic-gazebo-ros-control

conda install -c robostack ros-noetic-gazebo-ros-control 

# This is a command to install the "ros-noetic-gazebo-ros-control" package using Conda package manager.

# Here's what each part of the command means:

# conda is the command to interact with Conda package manager.
# install is the command to install a package.
# -c specifies the channel from which to install the package. In this case, the package is being installed from the "robostack" channel.
# ros-noetic-gazebo-ros-control is the name of the package being installed.
# This command will install the "ros-noetic-gazebo-ros-control" package from the "robostack" channel using Conda package manager.




# [Warning]: Some of the packages are not found at all
mamba install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

# [Important] Install TurtleBot3 Packages.!!! 

mamba install ros-kinetic-gazebo-ros

mamba install ros-noetic-dynamixel-sdk -y
mamba install ros-noetic-turtlebot3-msgs -y
mamba install ros-noetic-turtlebot3 -y
```



## Setup [TurtleBot3 Simulation Workflow](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
### Gazebo Simulation
1. Install Simulation packages
```Shell
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
cd src
```

M-Chip Mac + Parallel Ubuntu + [RoboStack + Gazebo + TurtleBot3] -- Installation



#


## Install Ubuntu
## Parallel Desktop 


## Intel Chip Mackbook


## [Try to] Install ROS on mac!
I am not sure whether it can be installed on M2 Mackbook Air, what I get the confidence by this [Available Packages Table](https://robostack.github.io/galactic.html).

### [RoboStack](https://robostack.github.io/index.html)

```Shell
# if you don't have mamba yet, install it first (not needed when using mambaforge):
conda install mamba -c conda-forge

# now create a new environment
# For ROS2 Humble
mamba create -n ros_env python=3.10 -c conda-forge
# For ROS1 Noetic and ROS2 Galactic -- [More common?]
mamba create -n ros_env python=3.9 -c conda-forge
conda activate ros_env

# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults

# Install the version of ROS you are interested in:

mamba install ros-noetic-desktop

# mamba install ros-humble-desktop  # (or "mamba install ros-noetic-desktop" or "mamba install ros-galactic-desktop")



# optionally, install some compiler packages if you want to e.g. build packages in a colcon_ws:
mamba install compilers cmake pkg-config make ninja colcon-common-extensions

# on Linux and osx (but not Windows) for ROS1 you might want to:
mamba install catkin_tools

# on Windows, install Visual Studio 2017 or 2019 with C++ support 
# see https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=msvc-160

# on Windows, install the Visual Studio command prompt:
mamba install vs2019_win-64

# note that in this case, you should also install the necessary dependencies with conda/mamba, if possible

# reload environment to activate required scripts before running anything
# on Windows, please restart the Anaconda Prompt / Command Prompt!
conda deactivate
conda activate ros_env

# if you want to use rosdep, also do:
mamba install rosdep
rosdep init  # note: do not use sudo!
rosdep update


```
