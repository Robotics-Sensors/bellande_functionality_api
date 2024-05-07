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
    
    # Get the parameters from the ROS parameter server
    x1 = rospy.get_param('x1')
    y1 = rospy.get_param('y1')
    x2 = rospy.get_param('x2')
    y2 = rospy.get_param('y2')
    limit = rospy.get_param('limit')

    # JSON payload
    payload = {
        "node0": {"x": x1, "y": y1},
        "node1": {"x": x2, "y": y2}
    }

    # Headers
    headers = {
        'accept': 'application/json',
        'Content-Type': 'application/json'
    }


    # Make POST request
    try:
        response = requests.post(
            url + endpoint_path + '?limit=' + str(limit),
            json=payload,
            headers=headers
        )
        response.raise_for_status()  # Raise an error for unsuccessful responses
        data = response.json()
        print("Next Step:", data['next_step'])
    except requests.exceptions.RequestException as e:
        print("Error:", e)

if __name__ == '__main__':
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        import rospy
    elif ros_version == "2":
        rclpy

    main()
