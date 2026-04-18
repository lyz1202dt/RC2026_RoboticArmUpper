from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    camera_source_arg = DeclareLaunchArgument(
        'camera_source',
        default_value='real',
        description='pnp_ros camera source: real or sim',
    )

    sim_image_topic_arg = DeclareLaunchArgument(
        'sim_image_topic',
        default_value='/camera_link/color/image_raw',
        description='Image topic used by pnp_ros in sim mode',
    )

    real_video_device_id_arg = DeclareLaunchArgument(
        'real_video_device_id',
        default_value='4',
        description='Video device id used by pnp_ros in real mode',
    )

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
            "0.1", "0.09", "-0.03",
            "0.0", "0.7071068", "0.0", "0.7071068",
            "link4",
            
            "camera_link"
        ]
    )

    static_tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
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
    # start_pnp = TimerAction(
    #     period=6.0,
    #     actions=[
    #         Node(
    #             package='pnp_ros',
    #             executable='pnp_ros_node',
    #             name='pnp_ros_node',
    #             output='screen',
    #             parameters=[{
    #                 'use_sim_time': False,
    #                 'camera_source': LaunchConfiguration('camera_source'),
    #                 'sim_image_topic': LaunchConfiguration('sim_image_topic'),
    #                 'real_video_device_id': LaunchConfiguration('real_video_device_id'),
    #             }],
    #         )
    #     ]
    # )

    return LaunchDescription([
        camera_source_arg,
        sim_image_topic_arg,
        real_video_device_id_arg,
        joint_driver,
        start_bringup,
        start_move_group,
        start_rviz,
        static_tf_link5_to_camera,
        static_tf_camera_to_optical,
        start_robotic_task,
        start_pnp,
    ])
