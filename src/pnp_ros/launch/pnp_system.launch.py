#!/usr/bin/env python3
"""
Launch文件：一键启动pnp_ros和robotic_arm系统
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
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

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.01', '0.01', '0',
            '0', '-1.5708', '0',
            'link5', 'camera_link'
        ]
    )

    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'camera_link', 'camera_optical_frame'
        ]
    )
    
    # pnp_ros节点 - 抓取控制节点
    pnp_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='pnp_ros',
                executable='pnp_ros_node',
                name='pnp_ros_node',
                output='screen',
                parameters=[{
                    'use_sim_time': False
                }]
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        static_tf,
        static_tf_2,
        pnp_node,
    ])
