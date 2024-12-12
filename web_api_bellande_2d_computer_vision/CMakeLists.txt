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

cmake_minimum_required(VERSION 3.8)
project(web_api_bellande_2d_computer_vision)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
  )
else()
  find_package(ament_cmake REQUIRED COMPONENTS
    rclcpp
    rclpy
  )
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    rospy
  )
endif()

# Install Python scripts for both ROS 1
if($ENV{ROS_VERSION} EQUAL 1)
  catkin_install_python(
    PROGRAMS
    src/bellande_2d_computer_vision_prediction.py
    src/bellande_2d_computer_vision_face_detection.py
    src/bellande_2d_computer_vision_object_detection.py
    src/bellande_2d_computer_vision_instance_segmentation.py
    src/bellande_2d_computer_vision_semantic_segmentation.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# Install Python scripts, configuration files, and launch files
if($ENV{ROS_VERSION} EQUAL "1")
    install(PROGRAMS src/bellande_2d_computer_vision_prediction.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(PROGRAMS src/bellande_2d_computer_vision_face_detection.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(PROGRAMS src/bellande_2d_computer_vision_object_detection.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(PROGRAMS src/bellande_2d_computer_vision_instance_segmentation.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(PROGRAMS src/bellande_2d_computer_vision_semantic_segmentation.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
    install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
else()
    install(PROGRAMS src/bellande_2d_computer_vision_prediction.py DESTINATION lib/${PROJECT_NAME})
    install(PROGRAMS src/bellande_2d_computer_vision_face_detection.py DESTINATION lib/${PROJECT_NAME})
    install(PROGRAMS src/bellande_2d_computer_vision_object_detection.py DESTINATION lib/${PROJECT_NAME})
    install(PROGRAMS src/bellande_2d_computer_vision_instance_segmentation.py DESTINATION lib/${PROJECT_NAME})
    install(PROGRAMS src/bellande_2d_computer_vision_semantic_segmentation.py DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
    install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
    ament_package()
endif()
