#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('vertical_lift_kit')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'vertical_lift_kit.urdf.xacro')
    
    # Path to controller config
    controller_config = os.path.join(pkg_dir, 'config', 'trajectory_controllers.yaml')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='empty.sdf')
    
    # Process the xacro file
    robot_description_content = Command(
        [
            'xacro', ' ', urdf_file
        ]
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )
    
    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            controller_config
        ]
    )
    
    # Ignition Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_name]
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_lift_kit',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'vertical_lift_kit',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Delayed controller spawning (wait for gazebo and controller manager to be ready)
    spawn_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    
    spawn_joint_trajectory_controller = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', '--type', 'joint_trajectory_cont' 'joint_trajectory_controller'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='empty.sdf',
            description='Gazebo world file name'
        ),
        gz_sim,
        robot_state_publisher_node,
        controller_manager_node,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller
    ])