#!/bin/bash

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
