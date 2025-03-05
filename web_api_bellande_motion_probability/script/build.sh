#!/bin/bash

# Create and navigate to build directory
mkdir -p build && cd build

# Build package
if [ $ROS_VERSION -eq 1 ]; then
    cmake -DROS_VERSION=1 ..
    make -j$(nproc)
else
    cmake -DROS_VERSION=2 ..
    make -j$(nproc)
fi

# Source package setup file
source devel/setup.bash

# Run rosdep
rosdep install --from-paths ../src --ignore-src -y

# Return to package root directory
cd ..

# Run ROS launch file
if [ $ROS_VERSION -eq 1 ]; then
    roslaunch web_api_bellande_step bellande_step_api_2d_launch.launch.py x1:=0 y1:=0 x2:=5 y2:=5 limit:=3
else
    ros2 launch web_api_bellande_step bellande_step_api_2d_launch.launch.py x1:=0 y1:=0 x2:=5 y2:=5 limit:=3
fi
