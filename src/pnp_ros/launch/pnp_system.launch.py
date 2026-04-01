#!/usr/bin/env python3
"""
Launch文件：一键启动pnp_ros和robotic_arm系统
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成launch描述"""
    
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
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        pnp_node,
    ])
