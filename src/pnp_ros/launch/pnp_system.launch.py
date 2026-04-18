#!/usr/bin/env python3
"""
Launch文件：一键启动pnp_ros和robotic_arm系统
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成launch描述"""
    camera_index = LaunchConfiguration('camera_index')
    frame_width = LaunchConfiguration('frame_width')
    frame_height = LaunchConfiguration('frame_height')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_frame_id = LaunchConfiguration('camera_frame_id')
    object_frame_id = LaunchConfiguration('object_frame_id')
    show_mask = LaunchConfiguration('show_mask')
    
    # 获取robotic_arm包的路径
    robotic_arm_share = get_package_share_directory('robotic_arm')
    urdf_file = os.path.join(robotic_arm_share, 'urdf', 'robotic_arm.urdf')
    
    # 读取URDF文件
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # robot_state_publisher节点 - 发布机器人模型
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # pnp_ros节点 - 抓取控制节点
    pnp_node = Node(
        package='pnp_ros',
        executable='pnp_ros_node',
        name='pnp_ros_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'camera_index': ParameterValue(camera_index, value_type=int),
            'frame_width': ParameterValue(frame_width, value_type=int),
            'frame_height': ParameterValue(frame_height, value_type=int),
            'camera_fps': ParameterValue(camera_fps, value_type=int),
            'camera_frame_id': camera_frame_id,
            'object_frame_id': object_frame_id,
            'show_mask': ParameterValue(show_mask, value_type=bool),
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='4'),
        DeclareLaunchArgument('frame_width', default_value='1280'),
        DeclareLaunchArgument('frame_height', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='60'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_link'),
        DeclareLaunchArgument('object_frame_id', default_value='target_object'),
        DeclareLaunchArgument('show_mask', default_value='true'),
        robot_state_publisher,
        pnp_node,
    ])
