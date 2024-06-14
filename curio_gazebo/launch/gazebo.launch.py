#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchIntrospector
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import DeclareLaunchArgument
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


    # Start up robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[gz_params],
    )
    ld.add_action(node_robot_state_publisher)

    # Controller to convert between /cmd_vel and /servo_controller/joint_trajectory and /wheel_controller/commands 
    controller_spawn = Node(
        package='curio_gazebo',
        executable='curio_controller',
        output='screen'
    )
    ld.add_action(controller_spawn)

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
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )
    ld.add_action(gz_ros2_bridge)

    #world = os.path.join(curio_gazebo_path, "worlds", "maze.world")
    world = LaunchConfiguration('world')
    ld.add_action(DeclareLaunchArgument('world',default_value=
            PathJoinSubstitution([curio_gazebo_path, "worlds", "smaze.world"])))

    #assert world.is_file(), "world file not found"

    # Launch gazebo environment
    #    -v 4 is verbose, level 4 (most)
    #    -r is run simultion on start
    #    world is last arg
    gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': [' -r -v 4 ', world], 'on_exit_shutdown': 'true'}.items()
            )
    ld.add_action(gzserver_cmd)

    # Now that gazebo is started, add the robot model
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

    # start up ignition, then controllers...
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_controller,
                         load_rover_wheel_controller,
                         load_servo_controller]
            )
    ))

    # Verbose introspection of launch object
    # print('Starting introspection of launch description...\n') 
    # print(LaunchIntrospector().format_launch_description(ld))
    # print('\nStarting launch of launch description...\n') 

    return ld
