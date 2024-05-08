import os
import sys
import subprocess


def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]

    # Construct the ROS 1 launch command
    roslaunch_command = ["roslaunch", "ros_web_api_bellande_step", "bellande_step_api_2d.launch"] + args

    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Declare launch arguments
    x1_arg = DeclareLaunchArgument('x1')
    y1_arg = DeclareLaunchArgument('y1')
    x2_arg = DeclareLaunchArgument('x2')
    y2_arg = DeclareLaunchArgument('y2')
    limit_arg = DeclareLaunchArgument('limit')

    # Create a list to hold all nodes to be launched
    nodes_to_launch = []

    # ROS2 specific configurations
    ros_launch_arguments = [
        x1_arg, y1_arg, x2_arg, y2_arg, limit_arg,
    ]
    nodes_to_launch.append(Node(
        package='ros_web_api_bellande_step',
        executable='bellande_step_api_2d.py',
        name='bellande_step_api_2d_node',
        output='screen',
        parameters=[
            {'x1': LaunchConfiguration('x1')},
            {'y1': LaunchConfiguration('y1')},
            {'x2': LaunchConfiguration('x2')},
            {'y2': LaunchConfiguration('y2')},
            {'limit': LaunchConfiguration('limit')},
        ],
    ))

    # Return the LaunchDescription containing all nodes and arguments
    return LaunchDescription(ros_launch_arguments + nodes_to_launch)

if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
