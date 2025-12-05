from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    pkg_dir = get_package_share_directory('robotic_arm')
    config_dir = get_package_share_directory('robotic_config')

    urdf_path = os.path.join(pkg_dir, "urdf", "robotic_arm.urdf")
    robot_desc = ParameterValue(Command(["xacro " + urdf_path]), value_type=str)

    controller_config = os.path.join(config_dir, "config", "ros2_controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config,
                    {"robot_description": robot_desc},
                    {
                        "robotic_arm_controller": {
                            "type": "joint_trajectory_controller/JointTrajectoryController"
                        },
                        "joint_state_broadcaster": {
                            "type": "joint_state_broadcaster/JointStateBroadcaster"
                        }
                    }],
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

        # robot_state_publisher：发布 /robot_description 和 TF
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     parameters=[{'robot_description': robot_desc}]
        # ),
        
        # ros2_control 主节点
        ros2_control_node,

        # 加载控制器
        joint_state_broadcaster_spawner,
        robotic_arm_controller_spawner
    ])
