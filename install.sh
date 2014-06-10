#!/bin/bash

# ros hydro install
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-hydro-desktop-full

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source /opt/ros/hydro/setup.bash

# install additional packages
sudo apt-get install ros-hydro-turtlebot-* ros-hydro-slam-gmapping

# initialize catkin ws
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash

# get program sources
sudo apt-get install git
cd src
git clone https://github.com/Alfeya/turtlebot_navigation

cd ~/catkin_ws
catkin_make

# make dependencies
sudo rosdep init
rosdep update

# get some models for gazebo playground
cd ~/
mkdir ~/.gazebo/models
wget http://gazebosim.org/models/dumpster/model.tar.gz .
tar xvf model.tar.gz -C ~/.gazebo/models
rm model.tar.gz

wget http://gazebosim.org/models/cube_20k/model.tar.gz .
tar xvf model.tar.gz -C ~/.gazebo/models
rm model.tar.gz

wget http://gazebosim.org/models/jersey_barrier/model.tar.gz .
tar xvf model.tar.gz -C ~/.gazebo/models
rm model.tar.gz
