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

    rvizconfig = LaunchConfiguration('rvizconfig')
    rvizconfig_arg = ld.add_action(DeclareLaunchArgument('rvizconfig',
        default_value=PathJoinSubstitution(
            [FindPackageShare('curio_viz'),'rviz','model.rviz'])))

    use_gui = LaunchConfiguration('use_gui')
    use_gui_arg = ld.add_action(DeclareLaunchArgument('use_gui',default_value="True"))

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

    rviz_cmd = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rvizconfig]
        )
    ld.add_action(rviz_cmd)

    return ld
