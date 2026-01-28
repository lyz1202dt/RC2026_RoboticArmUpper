from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_config").to_moveit_configs()
    

    controller_config_dir = get_package_share_directory('robotic_config')
    controller_config = os.path.join(controller_config_dir, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config,
                    moveit_config.robot_description,
                    {'hardware': 'arm_hw'}
        ],
        output="both"
    )

    # 延迟启动 spawner 以确保控制器管理器已准备好
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="both"
            )
        ]
    )

    robotic_arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["robotic_arm_controller", "--controller-manager", "/controller_manager"],
                output="both"
            )
        ]
    )

    return LaunchDescription([
        # ros2_control 主节点
        ros2_control_node,

        # 加载控制器
        joint_state_broadcaster_spawner,
        robotic_arm_controller_spawner
    ])
