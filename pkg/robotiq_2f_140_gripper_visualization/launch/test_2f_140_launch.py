from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths to important files
    robotiq_2f_140_description_path = get_package_share_directory('robotiq_2f_140_gripper_visualization')
    urdf_file = os.path.join(robotiq_2f_140_description_path, 'urdf', 'robotiq_arg2f_140_model.xacro')
    gazebo_world_file = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')  # You can change this to your custom world

    # Declare launch arguments
    return LaunchDescription([
        # Launch Gazebo simulator
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo',
            output='screen',
            arguments=[gazebo_world_file]
        ),

        # Launch Gazebo client for GUI (optional)
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_gui',
            output='screen',
            condition=LaunchConfiguration('gui')
        ),

        # Robot description parameter: load the URDF from xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),

        # Spawn the robot model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=['-entity', 'robotiq_2f_140', '-file', urdf_file]
        ),
    ])
