import os
import sys
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]

    # Construct the ROS 1 launch commandi
    roslaunch_command = ["roslaunch", "ros_web_api_bellande_3d_computer_vision", "bellande_3d_computer_vision_prediction.launch"] + args

    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    pass

if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
