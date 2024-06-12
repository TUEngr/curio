#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    ld = LaunchDescription()

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Set up paths
    curio_desc_path = get_package_share_directory('curio_description')
    curio_gazebo_path = get_package_share_directory('curio_gazebo')

    # Read in URDF
    xacro_file = os.path.join(curio_desc_path, 'urdf', 'curio.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    gz_params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'curio',
                   '-z', '0.25',
                   '-allow_renaming', 'true'],
    )
    ld.add_action(ignition_spawn_entity)

    # Start up robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[gz_params],
    )
    ld.add_action(node_robot_state_publisher)

    # joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # wheel_velocity_controller
    load_rover_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel_controller'],
        output='screen'
    )

    # servo_controller
    load_servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'servo_controller'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    ld.add_action(bridge)

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([curio_gazebo_path, 'worlds', 'maze.world']),
        description='World file to use in Gazebo')

    # Launch gazebo environment
    #    -v 4 is verbose, level 4 (most)
    #    -r is run simultion on start
    #    world is last arg
    ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py'])),
            launch_arguments=[('gz_args', [' -r -v 4 ', 
                        PathJoinSubstitution([curio_gazebo_path, 'worlds', 'maze.world'])])],
            ))

    # start up ignition, then controllers...
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_controller,
                         load_rover_wheel_controller,
                         load_servo_controller]
            )
    ))

    return ld
