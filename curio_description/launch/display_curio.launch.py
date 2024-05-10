#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    rover_path = get_package_share_path('curio_description')
    #rover_dir = str(rover_path)
    default_model_path = rover_path / 'urdf/curio.urdf'
    default_rviz_config_path = rover_path / 'rviz/urdf.rviz'
    #mesh_base_dir  = rover_dir  + "meshes/bases"
    #mesh_sensors_dir  = rover_dir  + "meshes/sensors"
    #mesh_wheels_dir  = rover_dir  + "meshes/wheels"

    # os.environ['GZ_SIM_RESOURCE_PATH'] = mesh_base_dir + ":" + mesh_sensors_dir

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
