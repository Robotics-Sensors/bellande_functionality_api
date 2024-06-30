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
import numpy as np
import base64
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray


def pointcloud_object_detection(cloud_msg):
    points = np.array(list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))))
    
    points_base64 = base64.b64encode(points.tobytes()).decode('utf-8')

    payload = {
        "pointcloud": points_base64
    }

    headers = {
        "Authorization": f"Bearer {api_access_key}"
    }

    response = requests.post(api_url, json=payload, headers=headers)

    if response.status_code == 200:
        result = response.json()
        marker_array = MarkerArray()
        for idx, obj in enumerate(result['objects']):
            marker = Marker()
            marker.header = cloud_msg.header
            marker.ns = "object_detection"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obj['centroid'][0]
            marker.pose.position.y = obj['centroid'][1]
            marker.pose.position.z = obj['centroid'][2]
            marker.scale.x = obj['dimensions'][0]
            marker.scale.y = obj['dimensions'][1]
            marker.scale.z = obj['dimensions'][2]
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        return marker_array
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None


def pointcloud_callback(msg):
    markers = pointcloud_object_detection(msg)
    if markers:
        pub.publish(markers)


def main():
    global api_url, api_access_key, pub

    config_file_path = os.path.join(os.path.dirname(__file__), '../config/configs.json')
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']["pointcloud_api"]
        api_access_key = config["Bellande_Framework_Access_Key"]
    
    if ros_version == "1":
        rospy.init_node('pointcloud_object_detection_node', anonymous=True)
        pub = rospy.Publisher('pointcloud_object_detection_result', MarkerArray, queue_size=10)
        sub = rospy.Subscriber('input_pointcloud', PointCloud2, pointcloud_callback)
    elif ros_version == "2":
        rclpy.init()
        node = rclpy.create_node('pointcloud_object_detection_node')
        pub = node.create_publisher(MarkerArray, 'pointcloud_object_detection_result', 10)
        sub = node.create_subscription(PointCloud2, 'input_pointcloud', pointcloud_callback, 10)

    api_url = f"{url}{endpoint_path}"

    try:
        print("Pointcloud object detection node is running. Ctrl+C to exit.")
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down pointcloud object detection node.")
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
