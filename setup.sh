#!/bin/bash

root_directory=$(pwd)

# Source ROS workspace (Overlays simulation workspace so it contains all simulation + ros packages)
source $root_directory/ros_ws/devel/setup.bash

