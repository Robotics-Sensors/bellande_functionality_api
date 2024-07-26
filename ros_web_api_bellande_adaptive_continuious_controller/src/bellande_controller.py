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
from std_msgs.msg import Float64

def bellande_controller(setpoint, current_value, kp, ki, kd):
    payload = {
        "setpoint": setpoint,
        "current_value": current_value,
        "kp": kp,
        "ki": ki,
        "kd": kd
    }
    headers = {
        "Authorization": f"Bearer {api_access_key}"
    }
    response = requests.post(api_url, json=payload, headers=headers)
    if response.status_code == 200:
        result = response.json()
        return result['control_output']
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None

def control_callback(msg):
    setpoint = rospy.get_param('setpoint', 0)
    kp = rospy.get_param('kp', 1.0)
    ki = rospy.get_param('ki', 0.1)
    kd = rospy.get_param('kd', 0.05)
    
    control_output = bellande_controller(setpoint, msg.data, kp, ki, kd)
    if control_output is not None:
        output_msg = Float64()
        output_msg.data = control_output
        pub.publish(output_msg)

def main():
    global api_url, api_access_key, pub
    config_file_path = os.path.join(os.path.dirname(__file__), '../config/configs.json')
    
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return
    
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']["bellande_controller"]
        api_access_key = config["Bellande_Framework_Access_Key"]
    
    # Initialize ROS node
    if ros_version == "1":
        rospy.init_node('bellande_controller_node', anonymous=True)
        pub = rospy.Publisher('control_output', Float64, queue_size=10)
        sub = rospy.Subscriber('current_value', Float64, control_callback)
    elif ros_version == "2":
        rclpy.init()
        node = rclpy.create_node('bellande_controller_node')
        pub = node.create_publisher(Float64, 'control_output', 10)
        sub = node.create_subscription(Float64, 'current_value', control_callback, 10)

    api_url = f"{url}{endpoint_path}"

    try:
        print("BellandeController node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down BellandeController node.")
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
