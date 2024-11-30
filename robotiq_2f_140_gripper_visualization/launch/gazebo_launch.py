#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import random
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package and file paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robotiq = get_package_share_directory('robotiq_2f_140_gripper_visualization')
    
    # World file
    world = os.path.join(pkg_robotiq, 'worlds', 'empty_world.world')
    
    # XACRO file details
    xacro_file = "robotiq_arg2f_140_model.xacro"
    robot_desc_path = os.path.join(pkg_robotiq, "urdf", xacro_file)
    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()
    
    # Robot spawn parameters
    position = [0.0, 0.0, 1.0]
    orientation = [0.0, 0.0, 0.0]
    robot_base_name = "robotiq_arg2f_140_model"
    entity_name = f"{robot_base_name}-{random.random()}"
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Nodes and launch files to include
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
                   '-topic', '/robot_description']
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    robot_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )
    
    robot_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )
    
    delay_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_position_controller_spawner],
        )
    )
    
    delay_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_velocity_controller_spawner],
        )
    )
    
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map', '/dummy_link'],
    )
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher,
        joint_state_publisher_node,
        spawn_robot,
        joint_state_broadcaster_spawner,
        delay_position_controller_spawner,
        delay_velocity_controller_spawner,
        tf_node
    ])