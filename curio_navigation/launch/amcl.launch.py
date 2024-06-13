#!/usr/bin/env python3
import os

#    <arg name="initial_pose_x" default="0.0" /> 
#    <arg name="initial_pose_y" default="0.0" /> 
#    <arg name="initial_pose_a" default="0.0" />
#
#    <node name="amcl" pkg="amcl" type="amcl"
#        respawn="false" output="screen">
#        <remap from="scan" to="sensors/laser" />

from pathlib import Path

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution 


def generate_launch_description():
    config = Path(get_package_share_directory('curio_naviation'), 'config')
    nav2_yaml = config / 'nav2.yaml'
    assert nav2_yaml.is_file()


    # Launch Arguments
    initial_pose_x = LaunchConfiguration('initial_pose_x', default=0.0)
    initial_pose_y = LaunchConfiguration('initial_pose_y', default=0.0)
    initial_pose_z = LaunchConfiguration('initial_pose_z', default=0.0)

    return LaunchDescription([
        launch.actions.SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        launch_ros.actions.Node(
            node_name='lifecycle_manager',
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            output='screen',
            parameters=[
                nav2_yaml,
                {
                    'autostart': True,
                    'node_names': ['amcl'],
                }
            ]
        ),
        launch_ros.actions.Node(
            node_name='amcl',
            package='nav2_amcl',
            node_executable='amcl',
            output='screen',
            parameters=[nav2_yaml],
            arguments=['__log_level:=debug']
          #  arguments=['__log_level:=debug']
        ),
    ])