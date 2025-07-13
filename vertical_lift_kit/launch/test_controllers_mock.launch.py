#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('vertical_lift_kit')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'vertical_lift_kit.urdf.xacro')
    
    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'lift_kit.rviz')
    
    # Path to controller config
    controller_config = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='true')
    
    # Process the xacro file with mock hardware
    robot_description_content = Command(
        [
            'xacro', ' ', urdf_file,
            ' use_mock_hardware:=', use_mock_hardware
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
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Delayed controller spawning (longer delays for mock hardware)
    spawn_joint_state_broadcaster = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    
    spawn_lift_controller = TimerAction(
        period=17.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'lift_position_controller'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='true',
            description='Use mock hardware if true'
        ),
        robot_state_publisher_node,
        controller_manager_node,
        rviz_node,
        spawn_joint_state_broadcaster,
        spawn_lift_controller
    ])