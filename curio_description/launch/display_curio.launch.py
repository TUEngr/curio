#!/usr/bin/env python3
#-- Display the robot model in rviz.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare 

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    model = LaunchConfiguration('model')
    model_arg = ld.add_action(DeclareLaunchArgument('model',
        default_value=PathJoinSubstitution(
            [FindPackageShare('curio_description'),'urdf','curio.urdf'])))

    use_gui = LaunchConfiguration('use_gui')
    use_gui_arg = ld.add_action(DeclareLaunchArgument('use_gui',default_value="True"))

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = ld.add_action(DeclareLaunchArgument('use_rviz',
        default_value='true',
        description='Whether to start RVIZ'))

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_file_cmd = ld.add_action(DeclareLaunchArgument('rviz_config_file',
    default_value=PathJoinSubstitution(
        [FindPackageShare('curio_viz'),'rviz','model.rviz']),
    description='Full path to the RVIZ config file to use'))

    # Launch RViz
    rviz_cmd = ld.add_action(Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file]))


    # Publish model of robot
    xacro_file = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    params = {"robot_description": xacro_file, "use_sim_time": True}

    robot_state_publisher_cmd = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )
    ld.add_action(robot_state_publisher_cmd)

    # Publish joints
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(use_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')
    start_joint_state_publisher_gui_cmd = Node(
        condition=IfCondition(use_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)

    return ld
