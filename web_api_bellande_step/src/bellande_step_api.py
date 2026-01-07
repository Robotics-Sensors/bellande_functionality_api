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

import json
import os
import requests

# Import Message
from web_api_bellande_step.msg import NextStep2D


# Config for Step Node
def get_next_step(x1, y1, x2, y2, limit):
    payload = {
        "node0": [x1, y1, 0],
        "node1": [x2, y2, 0],
        "limit": limit,
        "dimensions": 2,
        "auth": {"authorization_key": api_access_key},
    }

    headers = {
        "accept": "application/json",
        "Content-Type": "application/json",
    }

    try:
        response = requests.post(api_url, json=payload, headers=headers)
        response.raise_for_status()
        data = response.json()

        return data["next_step"]

    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return None


def parameter_callback_ros1(event):
    """ROS 1 timer callback with event parameter"""
    x1 = rospy.get_param("x1", 0)
    y1 = rospy.get_param("y1", 0)
    x2 = rospy.get_param("x2", 0)
    y2 = rospy.get_param("y2", 0)
    limit = rospy.get_param("limit", 3)

    next_step = get_next_step(x1, y1, x2, y2, limit)
    if next_step is not None:
        # Message Initialize
        msg = NextStep2D()
        msg.x = next_step[0]
        msg.y = next_step[1]

        pub.publish(msg)


def parameter_callback_ros2():
    """ROS 2 timer callback without event parameter"""
    x1 = node.get_parameter("x1").value
    y1 = node.get_parameter("y1").value
    x2 = node.get_parameter("x2").value
    y2 = node.get_parameter("y2").value
    limit = node.get_parameter("limit").value

    next_step = get_next_step(x1, y1, x2, y2, limit)
    if next_step is not None:
        # Message Initialize
        msg = NextStep2D()
        msg.x = next_step[0]
        msg.y = next_step[1]

        pub.publish(msg)


def node_initialize_ros1():
    rospy.init_node("next_step_node", anonymous=True)
    pub = rospy.Publisher("next_step_result", NextStep2D, queue_size=10)
    rospy.Timer(
        rospy.Duration(10), parameter_callback_ros1
    )  # Check parameters every 10 second

    return pub


def node_initialize_ros2():
    rclpy.init()
    node = rclpy.create_node("next_step_node")

    # Declare parameters for ROS 2
    node.declare_parameter("x1", 0)
    node.declare_parameter("y1", 0)
    node.declare_parameter("x2", 0)
    node.declare_parameter("y2", 0)
    node.declare_parameter("limit", 3)

    pub = node.create_publisher(NextStep2D, "next_step_result", 10)
    node.create_timer(10.0, parameter_callback_ros2)  # Check parameters every 10 second

    return node, pub


def get_config_file_path():
    """Get the correct config file path for ROS 1 or ROS 2"""
    if ros_version == "1":
        # ROS 1: Use rospack to find package share directory
        import rospkg

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("web_api_bellande_step")
        config_file_path = os.path.join(
            package_path, "config", "json", "http_configs.json"
        )
    elif ros_version == "2":
        # ROS 2: Use ament_index to find package share directory
        from ament_index_python.packages import get_package_share_directory

        package_share_directory = get_package_share_directory("web_api_bellande_step")
        config_file_path = os.path.join(
            package_share_directory, "config", "json", "http_configs.json"
        )
    else:
        raise ValueError(f"Unsupported ROS_VERSION: {ros_version}")

    return config_file_path


def main():
    global api_url, api_access_key, pub, node

    # Get config file path
    config_file_path = get_config_file_path()

    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return

    with open(config_file_path, "r") as config_file:
        config = json.load(config_file)
        url = config["url"]
        endpoint_path = config["endpoint_path"]["bellande_step"]
        api_access_key = config["Bellande_Framework_Access_Key"]

    # API URL
    api_url = f"{url}{endpoint_path}"

    # Initialize ROS node
    if ros_version == "1":
        pub = node_initialize_ros1()
    elif ros_version == "2":
        node, pub = node_initialize_ros2()

    try:
        print("Next step node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
        print("HEREE")
    except KeyboardInterrupt:
        print("Shutting down next step node.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        if ros_version == "2":
            node.destroy_node()
            rclpy.shutdown()

    print("HERE")


if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        import rospy
    elif ros_version == "2":
        import rclpy
    main()
