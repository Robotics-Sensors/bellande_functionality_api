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
        endpoint_path = config['endpoint_path']["2d"]
    

if __name__ == '__main__':
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        import rospy
    elif ros_version == "2":
        import rclpy

    main()
