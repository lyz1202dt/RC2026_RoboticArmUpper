from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    moveit_config_dir = get_package_share_directory("robotic_config")
    launch_dir = os.path.join(moveit_config_dir, "launch")
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_config").to_moveit_configs()

    bringup_launch = os.path.join(launch_dir, "bringup.launch.py")
    move_group_launch = os.path.join(launch_dir, "move_group.launch.py")
    rviz_launch = os.path.join(launch_dir, "moveit_rviz.launch.py")

    start_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch)
    )

    start_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch)
    )

    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch)
    )

    joint_driver=Node(
            package="robot_driver",
            executable="robot_driver",
            output="screen",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{
                "use_sim_time": False
            }])

    # Keep camera TF chain consistent with real_robot.launch.py.
    static_tf_link5_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.01', '0.01', '0',
            '0', '0', '0',
            'link5', 'camera_link'
        ]
    )

    static_tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'camera_link', 'camera_optical_frame'
        ]
    )

    # Delay PnP until TF tree and controllers are online.
    start_robotic_task = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robotic_task',
                executable='robotic_task',
                output='screen',
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ]
            )
        ]
    )

    # Start PnP after robotic_task is ready.
    start_pnp = TimerAction(
        period=6.0,
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
        joint_driver,
        start_bringup,
        start_move_group,
        start_rviz,
        static_tf_link5_to_camera,
        static_tf_camera_to_optical,
        start_robotic_task,
        start_pnp,
    ])
