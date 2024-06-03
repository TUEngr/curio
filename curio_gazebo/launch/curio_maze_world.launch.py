#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import AppendEnvironmentVariable
import xacro



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # ensure that gazebo can find all assets
    curio_desc_path = get_package_share_directory('curio_description')
    curio_gazebo_path = get_package_share_directory('curio_gazebo')
    models_path = curio_desc_path + "/meshes/bases:" + curio_desc_path + "/meshes/sensors:" + curio_desc_path + "/meshes/wheels"
    worlds_path = curio_gazebo_path + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(curio_desc_path,'meshes/bases'),
        os.path.join(curio_desc_path,'meshes/sensors'),
        os.path.join(curio_desc_path,'meshes/wheels'),
        os.path.join(curio_gazebo_path,'meshes/wheels')
        )

    urdf = os.path.join(curio_desc_path, 'urdf', 'curio.urdf')
    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    # robot_desc = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='maze.world',
        description='World file to use in Gazebo')
    
    world = LaunchConfiguration('world')
    gz_world_arg = PathJoinSubstitution([curio_gazebo_path, 'worlds', world])

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            'gz_args' : gz_world_arg,
            'on_exit_shutodown' : 'true'
        }.items()
    )
    
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            "-string", doc.toxml(),
            "-name", "curio",
            "-topic", "/robot_description",
            "-z", "0.25",
            "-allow_renaming", "true",
        ]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/sensors/laser@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
    )

    # Robot state publisher
    params = {'robot_description': doc.toxml()}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    # Controllers

    # joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # wheel_velocity_controller
    rover_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel_controller'],
        output='screen'
    )

    # servo_controller
    servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'servo_controller'],
        output='screen'
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(set_env_vars_resources)
    ld.add_action(declare_world_cmd)

    ld.add_action(gz_sim) # Launch Gazebo
    # ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)


    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    ld.add_action(RegisterEventHandler(
                event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[
                    load_joint_state_controller,
                    rover_wheel_controller,
                    servo_controller,
                ],
               )
            )
        ])

    return ld
