import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/pranavlakshmanan/ros2_ws/src/ur5_tutorial/urdf/ur5.urdf'

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
                output="screen",
            )
        ]
    )