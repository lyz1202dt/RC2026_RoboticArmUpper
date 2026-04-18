# ArUco 标记检测节点启动文件
# 用于手眼标定系统中的视觉标记追踪
# 
# 功能：
# 1. 从相机图像中检测指定 ID 的 ArUco 标记
# 2. 发布标记相对于相机坐标系的位置和姿态
# 3. 提供可视化工具在图像上绘制检测结果
#
# TF 树结构：
#   camera_optical_frame -> aruco_marker_frame
#
# 使用示例：
#   ros2 launch robotic_config aruco_detection.launch.py marker_id:=100 marker_size:=0.1

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== 参数声明 ====================
    
    # ArUco 标记 ID（范围取决于使用的字典，DICT_4X4_250 支持 0-249）
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='100',
        description='ArUco marker ID to detect'
    )
    
    # ArUco 标记的物理尺寸（单位：米）
    # 必须与实际打印/显示的标记尺寸一致，否则位姿估计会不准确
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1',
        description='ArUco marker size in meters'
    )
    
    # 标记位姿的参考坐标系
    # 通常设置为相机光学坐标系或机器人基座坐标系
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='base_link',
        description='Reference frame for marker pose'
    )
    
    # 角点优化方法，影响检测精度：
    # - NONE: 不进行优化（最快）
    # - HARRIS: 使用 Harris 角点检测
    # - LINES: 使用边缘线拟合（推荐，平衡速度和精度）
    # - SUBPIX: 亚像素级优化（最精确但较慢）
    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement',
        default_value='LINES',
        description='Corner refinement method: NONE, HARRIS, LINES, SUBPIX'
    )
    
    # ==================== 启动描述 ====================
    return LaunchDescription([
        # 注册所有可配置参数
        marker_id_arg,
        marker_size_arg,
        reference_frame_arg,
        corner_refinement_arg,
        
        # ==================== ArUco 单标记检测节点 ====================
        # 核心检测节点：订阅相机图像，检测指定的 ArUco 标记
        # 发布内容：
        #   - TF: camera_optical_frame -> aruco_marker_frame
        #   - Topic: /aruco_single/result (标记检测结果)
        Node(
            package='aruco_ros',
            executable='single',
            name='aruco_single',
            parameters=[{
                # 输入图像是否已经过畸变校正
                # MuJoCo 仿真相机通常输出无畸变图像，设为 True
                'image_is_rectified': True,
                
                # 标记物理尺寸（米），必须与实际标记一致
                'marker_size': LaunchConfiguration('marker_size'),
                
                # 要检测的标记 ID
                'marker_id': LaunchConfiguration('marker_id'),
                
                # 位姿发布的参考坐标系
                'reference_frame': LaunchConfiguration('reference_frame'),
                
                # 相机坐标系名称，必须与相机驱动发布的一致
                'camera_frame': 'camera_optical_frame',
                
                # 检测到的标记帧名称
                # 这将作为子帧发布到 TF 树：camera_optical_frame -> aruco_marker_frame
                'marker_frame': 'aruco_marker_frame',
                
                # 角点优化算法
                'corner_refinement': LaunchConfiguration('corner_refinement'),
                
                # 使用仿真时间（与 MuJoCo 同步）
                'use_sim_time': True
            }],
            # 话题重映射：将节点期望的话题映射到实际的相机话题
            remappings=[
                # 相机内参话题
                ('/camera_info', '/camera_link/color/camera_info'),
                # 原始图像话题
                ('/image', '/camera_link/color/image_raw'),
            ],
            output='screen'
        ),
        
        # ==================== ArUco 可视化节点（可选）====================
        # 在图像上绘制检测到的标记边框和 ID
        # 发布带标注的图像到 /aruco_marker_publisher/result 话题
        # 可用于调试和验证检测效果
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='aruco_marker_publisher',
            parameters=[{
                'use_sim_time': True
            }],
            remappings=[
                ('/camera_info', '/camera_link/color/camera_info'),
                ('/image', '/camera_link/color/image_raw'),
            ],
            output='screen'
        )
    ])