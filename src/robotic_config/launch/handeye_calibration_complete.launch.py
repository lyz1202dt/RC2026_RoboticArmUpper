from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='100',
        description='ArUco marker ID to detect'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1',
        description='ArUco marker size in meters'
    )
    
    show_rviz_arg = DeclareLaunchArgument(
        'show_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='true',
        description='Whether to show MuJoCo GUI window'
    )


    
    return LaunchDescription([
        # 参数声明
        marker_id_arg,
        marker_size_arg,
        show_rviz_arg,
        show_gui_arg,
        
        LogInfo(msg="Starting Hand-Eye Calibration System..."),
        
        # 1. 启动 MuJoCo 仿真环境
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotic_config'),
                    'launch',
                    'arm_mujoco_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'show_rviz': LaunchConfiguration('show_rviz'),
                'show_gui': LaunchConfiguration('show_gui'),
                'start_move_group': 'true',
                'start_camera': 'true',
                'start_camera_tf': 'true',
                'camera_source': 'sim',
            }.items()
        ),
        
        # 2. 启动手眼标定（包含 ArUco 检测）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotic_config'),
                    'launch',
                    'eye_in_hand.launch.py'
                ])
            ]),
            launch_arguments={
                'start_aruco': 'true',
                'marker_id': LaunchConfiguration('marker_id'),
                'marker_size': LaunchConfiguration('marker_size'),
            }.items()
        ),
        
        LogInfo(msg="Hand-Eye Calibration System started successfully!"),
        LogInfo(msg="Please move the robot arm to different poses and collect samples in the calibration GUI."),
    ])