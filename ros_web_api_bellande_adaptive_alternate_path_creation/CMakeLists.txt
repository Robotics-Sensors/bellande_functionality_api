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
project(ros_web_api_bellande_adaptive_alternate_path_creation)

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

# Install Python scripts, configuration files, and launch files
if($ENV{ROS_VERSION} EQUAL "1")
    install(PROGRAMS src/bellande_adaptive_alternate_path_creation_api_2d.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
    install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
elseif($ENV{ROS_VERSION} EQUAL "2")
    install(PROGRAMS src/bellande_adaptive_alternate_path_creation_api_2d.py DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
    install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()
