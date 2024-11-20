#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Find the paths of the relevant packages
    collectorbot_pkg = FindPackageShare('collectorbot')
    moveit_config_pkg = FindPackageShare('turtlebot3_manipulation_moveit_config')
    bringup_pkg = FindPackageShare('turtlebot3_manipulation_bringup')

    # Path to the custom world file in the CollectorBot package
    custom_world_path = PathJoinSubstitution(
        [
            collectorbot_pkg,
            'worlds',
            'new_world.world'
        ]
    )

    # Declare argument to control RViz startup
    rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether to start RViz2'
    )
    ld.add_action(rviz_arg)

    # Launch Gazebo with the custom world file
    gazebo_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_pkg, '/launch/gazebo.launch.py']),
        launch_arguments={
            'world': custom_world_path,
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )
    ld.add_action(gazebo_control_launch)

    # Include the move_group launch file to control the arm with MoveIt
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_config_pkg, '/launch/move_group.launch.py']),
        launch_arguments={
            'use_sim': 'true',
        }.items(),
    )
    ld.add_action(move_group_launch)

    # Include the MoveIt RViz configuration launch if RViz is to be started
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_config_pkg, '/launch/moveit_rviz.launch.py']),
        condition=IfCondition(LaunchConfiguration('start_rviz'))
    )
    ld.add_action(rviz_launch)

    # Use spawn_entity.py to spawn the TurtleBot3 robot into Gazebo
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_pkg, '/launch/hardware.launch.py']),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.0',
            'yaw': '0.0',
        }.items(),
    )
    ld.add_action(spawn_turtlebot)

    return ld
