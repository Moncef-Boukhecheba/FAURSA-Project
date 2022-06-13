#!/bin/bash

root_directory=$(pwd)

# Build Simulation workspace
cd $root_directory/simulation_ws 
rm -rf devel
catkin_make
source devel/setup.bash

# Build ROS workspace
cd $root_directory/ros_ws 
rm -rf devel
catkin_make
source devel/setup.bash

cd $root_directory

