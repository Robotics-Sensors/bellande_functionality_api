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
    args = sys.argv[1:]
    roslaunch_command = ["roslaunch", "web_api_bellande_adaptive_continuious_control_system", "bellande_control_system.launch"] + args
    subprocess.call(roslaunch_command)

def ros2_launch_description():
    initial_state_arg = DeclareLaunchArgument('initial_state', default_value='idle')
    name_arg = DeclareLaunchArgument('name', default_value='BellandeControlSystem')

    nodes_to_launch = []
    ros_launch_arguments = [
        initial_state_arg, name_arg,
    ]

    nodes_to_launch.append(Node(
        package='web_api_bellande_adaptive_continuious_control_system',
        executable='bellande_control_system.py',
        name='bellande_control_system_node',
        output='screen',
        parameters=[
            {'initial_state': LaunchConfiguration('initial_state')},
            {'name': LaunchConfiguration('name')},
        ],
    ))

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
