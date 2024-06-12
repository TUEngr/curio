#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([get_package_share_directory('curio_gazebo'),
                     
                              'launch', 'gz_sim.launch.py'])),
            launch_arguments=[('gz_args', [' -r -v 4 ', 
                        PathJoinSubstitution([curio_gazebo_path, 'worlds', 'maze.world'])])],
            ))



