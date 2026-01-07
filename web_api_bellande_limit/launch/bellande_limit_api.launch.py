#!/usr/bin/env python3
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


def generate_launch_description():
    """
    Required function for ROS 2 launch files.
    This is called by the ROS 2 launch system.
    """
    # Declare launch arguments
    x1_arg = DeclareLaunchArgument("x1", default_value="0")
    y1_arg = DeclareLaunchArgument("y1", default_value="0")
    x2_arg = DeclareLaunchArgument("x2", default_value="0")
    y2_arg = DeclareLaunchArgument("y2", default_value="0")
    e1_arg = DeclareLaunchArgument("e1", default_value="0")
    e2_arg = DeclareLaunchArgument("e2", default_value="0")
    s1_arg = DeclareLaunchArgument("s1", default_value="0")
    s2_arg = DeclareLaunchArgument("s2", default_value="0")
    g1_arg = DeclareLaunchArgument("g1", default_value="0")
    g2_arg = DeclareLaunchArgument("g2", default_value="0")
    p1_arg = DeclareLaunchArgument("p1", default_value="0")
    p2_arg = DeclareLaunchArgument("p2", default_value="0")
    d1_arg = DeclareLaunchArgument("d1", default_value="0")
    d2_arg = DeclareLaunchArgument("d2", default_value="0")
    sr_arg = DeclareLaunchArgument("sr", default_value="0")
    sp_arg = DeclareLaunchArgument("sp", default_value="0")

    # Create the node
    bellande_limit_node = Node(
        package="web_api_bellande_limit",
        executable="bellande_limit_api.py",
        name="bellande_limit_api_node",
        output="screen",
        parameters=[
            {"x1": LaunchConfiguration("x1")},
            {"y1": LaunchConfiguration("y1")},
            {"x2": LaunchConfiguration("x2")},
            {"y2": LaunchConfiguration("y2")},
            {"e1": LaunchConfiguration("e1")},
            {"e2": LaunchConfiguration("e2")},
            {"s1": LaunchConfiguration("s1")},
            {"s2": LaunchConfiguration("s2")},
            {"g1": LaunchConfiguration("g1")},
            {"g2": LaunchConfiguration("g2")},
            {"p1": LaunchConfiguration("p1")},
            {"p2": LaunchConfiguration("p2")},
            {"d1": LaunchConfiguration("d1")},
            {"d2": LaunchConfiguration("d2")},
            {"sr": LaunchConfiguration("sr")},
            {"sp": LaunchConfiguration("sp")},
        ],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        x1_arg,
        y1_arg,
        x2_arg,
        y2_arg,
        e1_arg,
        e2_arg,
        s1_arg,
        s2_arg,
        g1_arg,
        g2_arg,
        p1_arg,
        p2_arg,
        d1_arg,
        d2_arg,
        sr_arg,
        sp_arg,
        bellande_limit_node,
    ])


def ros1_launch_description():
    """
    For ROS 1 compatibility - call this directly if needed.
    """
    # Get command-line arguments
    args = sys.argv[1:]

    # Construct the ROS 1 launch command
    roslaunch_command = [
        "roslaunch",
        "web_api_bellande_limit",
        "bellande_limit_api.launch",
    ] + args

    # Execute the launch command
    subprocess.call(roslaunch_command)


if __name__ == "__main__":
    # This block only runs when executed directly (not when imported by ROS 2 launch)
    ros_version = os.getenv("ROS_VERSION")

    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        # For ROS 2, when run directly, we can manually trigger the launch
        print(
            "For ROS 2, use: ros2 launch web_api_bellande_limit bellande_limit_api.launch.py"
        )
        sys.exit(0)
    else:
        print(
            "Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2."
        )
        sys.exit(1)
