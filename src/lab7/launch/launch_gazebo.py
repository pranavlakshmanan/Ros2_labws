import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/pranavlakshmanan/ros2_ws/src/lab7/urdf/Two_wheeledBot.urdf'

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "Two_wheeledBot", "-b", "-file", urdf_file],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                arguments=[urdf_file],
                output="screen",
            ),
        ]
    )
