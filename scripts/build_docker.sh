#!/bin/bash

# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Check user input
if [ "$1" = "ros1" ]; then
    # Define the Dockerfile name and tag for ROS 1
    DOCKERFILE="../Dockerfile/Dockerfile.ros1"
    IMAGE_NAME="bellande_api_configs_packages_ros1"
elif [ "$1" = "ros2" ]; then
    # Define the Dockerfile name and tag for ROS 2
    DOCKERFILE="../Dockerfile/Dockerfile.ros2"
    IMAGE_NAME="bellande_api_configs_packages_ros2"
else
    echo "Invalid input. Please provide either 'ros1' or 'ros2'."
    exit 1
fi

TAG="latest"  # Change this to the desired tag for your Docker image

# Build the Docker image
docker build -t $IMAGE_NAME:$TAG -f $DOCKERFILE .

# Check if the image was built successfully
if [ $? -eq 0 ]; then
    echo "Docker image $IMAGE_NAME:$TAG built successfully."
else
    echo "Error: Failed to build Docker image."
fi
