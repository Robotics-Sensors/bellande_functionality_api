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

import os
import sys
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable


def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]
    
    # Construct the ROS 1 launch command
    roslaunch_command = ["roslaunch", "ros_web_api_bellande_3d_computer_vision", "bellande_3d_computer_vision_prediction.launch"] + args
    
    # Add the pointcloud publisher node
    roslaunch_command.extend([
        "ros_web_api_bellande_3d_computer_vision", "pointcloud_pradiction_node", "name:=pointcloud_publisher",
        "frame_id:=base_link"
    ])
    
    # Add the prediction node
    roslaunch_command.extend([
        "ros_web_api_bellande_3d_computer_vision", "bellande_3d_computer_vision_prediction.py", "name:=pointcloud_prediction_node"
    ])
    
    # Add the rviz node
    roslaunch_command.extend([
        "rviz", "rviz", "name:=rviz",
        "args:=-d $(find ros_web_api_bellande_3d_computer_vision)/rviz/pointcloud_visualization.rviz"
    ])
    
    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Create a list to hold all nodes to be launched
    nodes_to_launch = []
    
    # Add the pointcloud publisher node
    nodes_to_launch.append(Node(
        package='ros_web_api_bellande_3d_computer_vision',
        executable='pointcloud_pradiction_node',
        name='pointcloud_publisher',
        output='screen',
        parameters=[{'frame_id': 'base_link'}]
    ))
    
    # Add the prediction node
    nodes_to_launch.append(Node(
        package='ros_web_api_bellande_3d_computer_vision',
        executable='bellande_3d_computer_vision_prediction.py',
        name='pointcloud_prediction_node',
        output='screen',
        remappings=[('input_pointcloud', '/pointcloud_topic')]
    ))
    
    # Add the rviz node
    nodes_to_launch.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '$(find ros_web_api_bellande_3d_computer_vision)/rviz/pointcloud_visualization.rviz']
    ))
    
    # Return the LaunchDescription containing all nodes
    return LaunchDescription(nodes_to_launch)


if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
