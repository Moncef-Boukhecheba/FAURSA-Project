#!/bin/bash

# Install dependencies
sudo apt install ros-noetic-map-server ros-noetic-amcl ros-noetic-gmapping

# Build workspaces and source setup files 
source make_projects.sh
