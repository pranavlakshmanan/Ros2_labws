import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_project = get_package_share_directory('project')
    
    # Path to the world file
    world_file = os.path.join(pkg_project, 'worlds', 'arena.world')
    
    # Path to the URDF/Xacro file
    urdf_file = os.path.join(pkg_project, 'urdf', 'Robot.urdf.xacro')
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Create temp directory for URDF files
    temp_dir = tempfile.mkdtemp()
    
    # Process and save Robot 1 (Red)
    Robot1_name = "Robot1"
    Robot1_color = "1 0 0 1"
    Robot1_body_color = "Red"
    Robot1_wheel_color = "Orange"
    Robot1_racket_color = "Orange"

    doc1 = xacro.process_file(urdf_file, mappings={
        'robot_name': Robot1_name, 
        'Robot_color': Robot1_color,
        'body_color': Robot1_body_color,
        'wheel_color': Robot1_wheel_color,
        'racket_color': Robot1_racket_color
    })
    Robot1_urdf = os.path.join(temp_dir, f"{Robot1_name}.urdf")
    with open(Robot1_urdf, 'w') as f:
        f.write(doc1.toxml())
    
    # Robot 1 State Publisher
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot1_state_publisher',
        output='screen',
        parameters=[{'robot_description': doc1.toxml()}],
        remappings=[('/joint_states', f'/{Robot1_name}/joint_states')]
    )

    # Robot 1 Joint State Publisher
    robot1_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='robot1_joint_state_publisher',
        parameters=[{'source_list': [f'/{Robot1_name}/joint_states']}],
        output='screen'
    )
    
    spawn_Robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', Robot1_urdf,
            '-entity', Robot1_name,
            '-robot_namespace', Robot1_name,
            '-x', '-4.5',
            '-y', '0',
            '-z', '0.51'
        ],
        output='screen'
    )
    
    # Process and save Robot 2 (Blue)
    Robot2_name = "Robot2"
    Robot2_color = "0 0 1 1"
    Robot2_body_color = "Blue"
    Robot2_wheel_color = "Turquoise"
    Robot2_racket_color = "Turquoise"
    
    doc2 = xacro.process_file(urdf_file, mappings={
        'robot_name': Robot2_name, 
        'Robot_color': Robot2_color,
        'body_color': Robot2_body_color,
        'wheel_color': Robot2_wheel_color,
        'racket_color': Robot2_racket_color
    })
    Robot2_urdf = os.path.join(temp_dir, f"{Robot2_name}.urdf")
    with open(Robot2_urdf, 'w') as f:
        f.write(doc2.toxml())
    
    # Robot 2 State Publisher
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot2_state_publisher',
        output='screen',
        parameters=[{'robot_description': doc2.toxml()}],
        remappings=[('/joint_states', f'/{Robot2_name}/joint_states')]
    )

    # Robot 2 Joint State Publisher
    robot2_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='robot2_joint_state_publisher',
        parameters=[{'source_list': [f'/{Robot2_name}/joint_states']}],
        output='screen'
    )
    
    spawn_Robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', Robot2_urdf,
            '-entity', Robot2_name,
            '-robot_namespace', Robot2_name,
            '-x', '4.5',
            '-y', '0',
            '-z', '0.51',
            '-Y', '3.14159'
        ],
        output='screen'
    )
    
    # Joy Node for Robot2
    joy_node = Node(
        package='joy',
        executable='joy_node',
        namespace='Robot2',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Combined Teleop Node for both robots
    mecanum_teleop_node = Node(
        package='project',  # Replace with your actual package name
        executable='telop',
        name='mecanum_teleop_node',
        output='screen',
        parameters=[{
            'linear_vel': 0.5,
            'angular_vel': 1.57,
            'vel_step': 0.1
        }]
    )
    
    return LaunchDescription([
        gazebo,
        robot1_state_publisher,
        robot1_joint_state_publisher,
        spawn_Robot1,
        robot2_state_publisher,
        robot2_joint_state_publisher,
        spawn_Robot2,
        joy_node,
        mecanum_teleop_node
    ])