from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # === 修改这里：为你的 MoveIt 配置包名称 ===
    joint_driver=Node(
            package="robot_driver",
            executable="robot_driver",
            name="real_joint_state_pub",
            output="screen",
            parameters=[{
                "use_sim_time": False
            }]
        )

    
    moveit_config_dir = get_package_share_directory("robotic_config")


    launch_dir = os.path.join(moveit_config_dir, "launch")
    # === 加载 MoveIt Setup Assistant 自动生成的 launch 文件 ===
    move_group_launch = os.path.join(launch_dir, "move_group.launch.py")
    rviz_launch        = os.path.join(launch_dir, "moveit_rviz.launch.py")
    # 其他文件也能随时加载，比如 sensor_manager.launch.py

    move_group=IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch))

    rviz_show=IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz_launch))


    return LaunchDescription([joint_driver,move_group,rviz_show])
