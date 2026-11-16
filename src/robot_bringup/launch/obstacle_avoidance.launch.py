import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Find the TurtleBot3 Gazebo package
    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    # Include the existing TurtleBot3 simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_dir,
                'launch',
                'turtlebot3_world.launch.py'
            )
        ),
        # launch_arguments={
        #     'x_pose': '0.0',
        #     'y_pose': '0.0',
        # }.items()
    )

    # Start our obstacle avoidance controller
    obstacle_avoidance_node = Node(
        package='obstacle_avoidance',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[{
            'safe_distance': 0.6,
            'emergency_distance': 0.25,
            'forward_speed': 0.15,
            'turn_speed': 0.6
        }]
    )

    return LaunchDescription([
        gazebo_launch,
        obstacle_avoidance_node
    ])