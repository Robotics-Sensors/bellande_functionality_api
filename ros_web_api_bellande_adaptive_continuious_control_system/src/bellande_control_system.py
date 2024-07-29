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

def bellande_control_system(state, action, parameters):
    payload = {
        "state": state,
        "action": action,
        "parameters": parameters
    }
    headers = {
        "Authorization": f"Bearer {api_access_key}",
        "X-Connectivity-Passcode": connectivity_passcode
    }
    response = requests.post(api_url, json=payload, headers=headers)
    if response.status_code == 200:
        result = response.json()
        return result['output'], result.get('next_state')
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None, None

def control_callback(msg):
    data = json.loads(msg.data)
    current_state = rospy.get_param('current_state', 'idle')
    action = data['action']
    parameters = data['parameters']
    
    output, next_state = bellande_control_system(current_state, action, parameters)
    if output is not None:
        output_msg = String()
        output_msg.data = json.dumps({
            "output": output,
            "next_state": next_state
        })
        pub.publish(output_msg)
        if next_state:
            rospy.set_param('current_state', next_state)

def main():
    global api_url, api_access_key, connectivity_passcode, pub
    config_file_path = os.path.join(os.path.dirname(__file__), '../config/configs.json')
    
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return
    
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']["bellande_control_system_base"]
        connectivity_passcode = config["connectivity_passcode"]
        api_access_key = config["Bellande_Framework_Access_Key"]
    
    # Initialize ROS node
    if ros_version == "1":
        rospy.init_node('bellande_control_system_node', anonymous=True)
        pub = rospy.Publisher('bellande_control_system_node_control_output', String, queue_size=10)
        sub = rospy.Subscriber('bellande_control_system_node_control_action', String, control_callback)
    elif ros_version == "2":
        rclpy.init()
        node = rclpy.create_node('bellande_control_system_node')
        pub = node.create_publisher(String, 'bellande_control_system_node_control_output', 10)
        sub = node.create_subscription(String, 'bellande_control_system_node_control_action', control_callback, 10)
    
    api_url = f"{url}{endpoint_path}"
    
    try:
        print("BellandeControlSystem node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down BellandeControlSystem node.")
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
