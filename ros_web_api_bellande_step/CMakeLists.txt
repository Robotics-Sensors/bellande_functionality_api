cmake_minimum_required(VERSION 3.8)
project(ros_web_api_bellande_step)

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
    install(PROGRAMS src/bellande_step_api_2d.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
    install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
elseif($ENV{ROS_VERSION} EQUAL "2")
    install(PROGRAMS src/bellande_step_api_2d.py DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
    install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()
