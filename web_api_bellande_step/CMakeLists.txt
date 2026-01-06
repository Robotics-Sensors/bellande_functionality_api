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
project(web_api_bellande_step)

# Find ROS
if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
  )
  
  add_message_files(
    FILES
    NextStep2D.msg
  )

  # This enables Python package installation via setup.py
  catkin_python_setup()
  
  catkin_package(
    CATKIN_DEPENDS
      roscpp
      rospy
  )
  
  # Install config files
  install(
    DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )
  
  # Install launch files
  install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
  
else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/NextStep2D.msg"
  )

  # Install Python scripts directly (setup.py handles entry points)
  install(
    PROGRAMS src/bellande_step_api.py
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install config files
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  
  # Install launch files
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  
  ament_package()
endif()
