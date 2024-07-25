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

def main():
    # Get the absolute path to the config file
    config_file_path = os.path.join(os.path.dirname(__file__), '../config/configs.json')
    
    # Check if the config file exists
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return

    # Read configuration from config.json
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']["detection"]
    
    # Get the parameters from the ROS parameter server
    audio_data = rospy.get_param('audio_data', '')
    sample_rate = rospy.get_param('sample_rate', 16000)
    language = rospy.get_param('language', 'en-US')

    # JSON payload
    payload = {
        "audio_data": audio_data,
        "sample_rate": sample_rate,
        "language": language
    }

    # Headers
    headers = {
        'accept': 'application/json',
        'Content-Type': 'application/json'
    }

    # Make POST request
    try:
        response = requests.post(
            url + endpoint_path,
            json=payload,
            headers=headers
        )
        response.raise_for_status()  # Raise an error for unsuccessful responses
        data = response.json()
        print("Detected Speech:", data['detected_speech'])
    
    except requests.exceptions.RequestException as e:
        print("Error:", e)

if __name__ == '__main__':
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        import rospy
    elif ros_version == "2":
        import rclpy
    main()
