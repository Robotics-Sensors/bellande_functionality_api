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

def speech_detection(audio_data, sample_rate, language):
    payload = {
        "audio_data": audio_data,
        "sample_rate": sample_rate,
        "language": language
    }
    headers = {
        "Authorization": f"Bearer {api_access_key}",
        'accept': 'application/json',
        'Content-Type': 'application/json'
    }
    response = requests.post(api_url, json=payload, headers=headers)
    if response.status_code == 200:
        result = response.json()
        return result['detected_speech']
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None

def audio_callback(msg):
    sample_rate = rospy.get_param('sample_rate', 16000)
    language = rospy.get_param('language', 'en-US')
    
    detected_speech = speech_detection(msg.data, sample_rate, language)
    if detected_speech is not None:
        output_msg = String()
        output_msg.data = detected_speech
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
        endpoint_path = config['endpoint_path']["speech_detection"]
        api_access_key = config["Bellande_Framework_Access_Key"]
    
    # Initialize ROS node
    if ros_version == "1":
        rospy.init_node('speech_detection_node', anonymous=True)
        pub = rospy.Publisher('detected_speech', String, queue_size=10)
        sub = rospy.Subscriber('audio_data', String, audio_callback)
    elif ros_version == "2":
        rclpy.init()
        node = rclpy.create_node('speech_detection_node')
        pub = node.create_publisher(String, 'detected_speech', 10)
        sub = node.create_subscription(String, 'audio_data', audio_callback, 10)

    api_url = f"{url}{endpoint_path}"

    try:
        print("Speech detection node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down speech detection node.")
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
