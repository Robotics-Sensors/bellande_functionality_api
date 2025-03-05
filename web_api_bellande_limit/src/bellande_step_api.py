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
from std_msgs.msg import String

# Import Bellande
from bellande_step import *

def get_next_step(x1, y1, x2, y2, limit):
    payload = {
        "node0": {"x": x1, "y": y1},
        "node1": {"x": x2, "y": y2}
    }

    headers = {
        'accept': 'application/json',
        'Content-Type': 'application/json',
        "Authorization": f"Bearer {api_access_key}"
    }

    try:
        response = requests.post(
            f"{api_url}?limit={limit}",
            json=payload,
            headers=headers
        )
        response.raise_for_status()
        data = response.json()
        return String(f"Next Step: {data['next_step']}")
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return None

def parameter_callback(event):
    x1 = rospy.get_param('x1', 0)
    y1 = rospy.get_param('y1', 0)
    x2 = rospy.get_param('x2', 0)
    y2 = rospy.get_param('y2', 0)
    limit = rospy.get_param('limit', 3)

    next_step = get_next_step(x1, y1, x2, y2, limit)
    if next_step:
        pub.publish(next_step)

def main():
    global api_url, api_access_key, pub

    config_file_path = os.path.join(os.path.dirname(__file__), '../config/configs.json')
    
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return
    
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']["2d"]
        api_access_key = config["Bellande_Framework_Access_Key"]
    
    # Initialize ROS node
    if ros_version == "1":
        rospy.init_node('next_step_node', anonymous=True)
        pub = rospy.Publisher('next_step_result', String, queue_size=10)
        rospy.Timer(rospy.Duration(1), parameter_callback)  # Check parameters every second
    elif ros_version == "2":
        rclpy.init()
        node = rclpy.create_node('next_step_node')
        pub = node.create_publisher(String, 'next_step_result', 10)
        node.create_timer(1.0, parameter_callback)  # Check parameters every second

    api_url = f"{url}{endpoint_path}"

    try:
        print("Next step node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down next step node.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        if ros_version == "2":
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        import rospy
    elif ros_version == "2":
        import rclpy
    main()
