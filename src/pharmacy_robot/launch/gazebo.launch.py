#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Import Node for spawning models

# generate launch description
def generate_launch_description():

    # Get Gazebo ROS interface package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Get the location for empty world
    world = os.path.join(
        get_package_share_directory('pharmacy_robot'),
        'worlds',
        'pr_world.world'
    )

    # Set the path to your custom model directory
    custom_model_path = os.path.join(
        get_package_share_directory('pharmacy_robot'),
        'models'
    )
    
    # Add the custom model path to GAZEBO_MODEL_PATH
    os.environ['GAZEBO_MODEL_PATH'] = custom_model_path

    # Launch Description to run Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Launching the  Description to run Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Getting the package directory 
    pkg_gazebo = get_package_share_directory('pharmacy_robot')

    # Launching the Description to Spawn Robot Model 
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch',
                         'spawn_robot_ros2.launch.py'),
        )
    )

    # # Node to spawn the custom model into the Gazebo simulation
    # spawn_custom_model = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'bookshelf_custom',  # Replace with your model name
    #         '-file', os.path.join(custom_model_path, 'bookshelf_custom', 'model.sdf'),  # Path to the SDF file
    #         '-x', '0', '-y', '0', '-z', '0'  # Initial position of the model
    #     ],
    #     output='screen'
    # )

    # Launch Description 
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_world,
        #spawn_custom_model  # Add the custom model spawning here
    ])

