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
import cv2
import base64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


def predict(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

    _, img_encoded = cv2.imencode('.jpg', cv_image)
    img_base64 = base64.b64encode(img_encoded).decode('utf-8')

    payload = {
        "image": img_base64
    }

    response = requests.post(api_url, json=payload)

    if response.status_code == 200:
        result = response.json()
        prediction = result['prediction']
        confidence = result['confidence']
        
        prediction_msg = f"Prediction: {prediction}, Confidence: {confidence:.2f}"
        return String(prediction_msg)
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None


def image_callback(msg):
    prediction = predict(msg)
    if prediction:
        pub.publish(prediction)


def main():
    global api_url, pub

    config_file_path = os.path.join(os.path.dirname(__file__), '../config/configs.json')
    
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return
    
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']["prediction"]
    
    # Initialize ROS node
    if ros_version == "1":
        rospy.init_node('prediction_node', anonymous=True)
        pub = rospy.Publisher('prediction_result', String, queue_size=10)
        sub = rospy.Subscriber('camera/image_raw', Image, image_callback)
    elif ros_version == "2":
        rclpy.init()
        node = rclpy.create_node('prediction_node')
        pub = node.create_publisher(String, 'prediction_result', 10)
        sub = node.create_subscription(Image, 'camera/image_raw', image_callback, 10)

    api_url = f"{url}{endpoint_path}"

    try:
        print("Prediction node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down prediction node.")
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
