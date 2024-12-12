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

def get_ai_response(input_text):
    payload = {
        "input_text": input_text
    }
    headers = {
        'accept': 'application/json',
        'Content-Type': 'application/json',
        "Authorization": f"Bearer {api_access_key}"
    }
    try:
        response = requests.post(
            api_url,
            json=payload,
            headers=headers
        )
        response.raise_for_status()
        data = response.json()
        return String(f"AI Response: {data['response']}")
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return None

def input_callback(msg):
    ai_response = get_ai_response(msg.data)
    if ai_response:
        pub.publish(ai_response)

def main():
    global api_url, api_access_key, pub
    config_file_path = os.path.join(os.path.dirname(__file__), '../config/configs.json')
    
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return
    
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']["base"]
        api_access_key = config["Bellande_Framework_Access_Key"]
    
    # Initialize ROS node
    if ros_version == "1":
        rospy.init_node('ai_system_node', anonymous=True)
        pub = rospy.Publisher('ai_system_node_ai_response', String, queue_size=10)
        sub = rospy.Subscriber('ai_system_node_ai_input', String, input_callback)
    elif ros_version == "2":
        rclpy.init()
        node = rclpy.create_node('ai_system_node')
        pub = node.create_publisher(String, 'ai_system_node_ai_response', 10)
        sub = node.create_subscription(String, 'ai_system_node_ai_input', input_callback, 10)
    
    api_url = f"{url}{endpoint_path}"
    
    try:
        print("AI system node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down AI system node.")
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
