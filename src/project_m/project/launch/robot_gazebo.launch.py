import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('project')

    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'Robot.urdf')

    # Process the URDF file with xacro
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "robot", "-b", "-topic", "/robot_description"],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{'robot_description': robot_description}]
            ),
        ]
    )