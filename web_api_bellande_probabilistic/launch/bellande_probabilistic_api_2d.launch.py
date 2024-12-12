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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]

    # Construct the ROS 1 launch commandi
    roslaunch_command = ["roslaunch", "web_api_bellande_probabilistic", "bellande_probabilistic_api_2d.launch"] + args

    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Declare launch arguments
    x1_arg = DeclareLaunchArgument('x1')
    y1_arg = DeclareLaunchArgument('y1')
    x2_arg = DeclareLaunchArgument('x2')
    y2_arg = DeclareLaunchArgument('y2')
    limit_arg = DeclareLaunchArgument('limit')

    # Create a list to hold all nodes to be launched
    nodes_to_launch = []

    # ROS2 specific configurations
    ros_launch_arguments = [
        x1_arg, y1_arg, x2_arg, y2_arg, limit_arg,
    ]
    nodes_to_launch.append(Node(
        package='web_api_bellande_probabilistic',
        executable='bellande_probabilistic_api_2d.py',
        name='bellande_probabilistic_api_2d_node',
        output='screen',
        parameters=[
            {'x1': LaunchConfiguration('x1')},
            {'y1': LaunchConfiguration('y1')},
            {'x2': LaunchConfiguration('x2')},
            {'y2': LaunchConfiguration('y2')},
            {'limit': LaunchConfiguration('limit')},
        ],
    ))

    # Return the LaunchDescription containing all nodes and arguments
    return LaunchDescription(ros_launch_arguments + nodes_to_launch)

if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
