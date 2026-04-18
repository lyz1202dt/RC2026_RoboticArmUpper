from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    start_aruco_arg = DeclareLaunchArgument(
        'start_aruco',
        default_value='true',
        description='是否启动 ArUco 标记检测'
    )
    
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='100',
        description='ArUco marker ID'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1',
        description='ArUco marker size in meters'
    )
    
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='camera_optical_frame',
        description='Reference frame for marker pose output'
    )
    
    return LaunchDescription([
        start_aruco_arg,
        marker_id_arg,
        marker_size_arg,
        reference_frame_arg,
        
        # 启动 ArUco 标记检测
        # 发布 TF: camera_optical_frame -> aruco_marker_frame
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotic_config'),
                    'launch',
                    'aruco_detection.launch.py'
                ])
            ]),
            launch_arguments={
                'marker_id': LaunchConfiguration('marker_id'),
                'marker_size': LaunchConfiguration('marker_size'),
                'reference_frame': LaunchConfiguration('reference_frame'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_aruco'))
        ),
        
        # 启动手眼标定
        # 注意：需要先运行 MuJoCo 仿真 (arm_mujoco_sim.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('easy_handeye2'),
                    'launch',
                    'calibrate.launch.py'
                ])
            ]),
            launch_arguments={
                'calibration_type': 'eye_in_hand',
                'name': 'my_eih_calib',
                
                # 机器人的 TF 帧
                'robot_base_frame': 'base_link',
                # 机械臂末端执行器帧
                'robot_effector_frame': 'link6',
                
                # 追踪系统的 TF 帧
                # 使用 camera_optical_frame 与 aruco_ros 输出父帧保持一致
                # aruco_marker_frame 是检测到的标记坐标系
                'tracking_base_frame': 'camera_optical_frame',
                'tracking_marker_frame': 'aruco_marker_frame',

                # 仿真已经提供真实相机TF，不再使用easy_handeye的dummy TF
                'publish_dummy': 'false',
                'use_sim_time': 'true',
                
                'freehand_robot_movement': 'true'
            }.items()
        )
    ])