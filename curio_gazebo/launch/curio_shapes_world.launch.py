#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution 

def generate_launch_description():
    ld = LaunchDescription()

    # Launch Augments
    world = LaunchConfiguration('world')

    curio_gazebo_path = get_package_share_directory('curio_gazebo')
    ld.add_action(DeclareLaunchArgument('world',default_value=
            PathJoinSubstitution([curio_gazebo_path, "worlds", "shapes.world"])))

    # Launch gazebo with substituted world
    gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([get_package_share_directory('curio_gazebo'), 'launch', 'gazebo.launch.py'])),
            launch_arguments={'world': world}.items()
            )
    ld.add_action(gzserver_cmd)

    return ld
