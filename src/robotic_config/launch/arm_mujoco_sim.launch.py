from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    robotic_arm_share = get_package_share_directory("robotic_arm")
    robotic_config_share = get_package_share_directory("robotic_config")
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_config").to_moveit_configs()
    arm_joint_states_topic = "/joint_state_broadcaster/joint_states"

    urdf_path = os.path.join(robotic_arm_share, "urdf", "robotic_arm_mujoco.urdf")
    controller_yaml = os.path.join(robotic_config_share, "config", "ros2_controllers_mujoco.yaml")
    rviz_path = os.path.join(robotic_config_share, "config", "moveit.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    default_mjcf = os.path.join(robotic_arm_share, "model", "scene.xml")

    mjcf_path_arg = DeclareLaunchArgument(
        "mjcf_path",
        default_value=default_mjcf,
        description="Path to the robotic arm MJCF/scene XML used by MuJoCo",
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with MuJoCo simulation",
    )

    show_gui_arg = DeclareLaunchArgument(
        "show_gui",
        default_value="true",
        description="Whether to show MuJoCo GUI window",
    )

    start_arm_calc_arg = DeclareLaunchArgument(
        "start_arm_calc",
        default_value="false",
        description="Whether to start arm_calc node (it publishes myjoints_target and may override external commands)",
    )

    start_move_group_arg = DeclareLaunchArgument(
        "start_move_group",
        default_value="true",
        description="Whether to start move_group together with MuJoCo simulation",
    )

    start_robotic_task_arg = DeclareLaunchArgument(
        "start_robotic_task",
        default_value="true",
        description="Whether to start robotic_task node",
    )

    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Whether to start pnp_ros node for real camera visual target publishing",
    )

    start_camera_tf_arg = DeclareLaunchArgument(
        "start_camera_tf",
        default_value="true",
        description="Whether to publish static TF from link5 to camera_link",
    )
    
    # 真实相机相对于link5的偏移（根据实际安装位置调整）
    camera_x_arg = DeclareLaunchArgument(
        "camera_x",
        default_value="0.01",
        description="Camera X offset from link5 (meters)",
    )
    camera_y_arg = DeclareLaunchArgument(
        "camera_y",
        default_value="0.01",
        description="Camera Y offset from link5 (meters)",
    )
    camera_z_arg = DeclareLaunchArgument(
        "camera_z",
        default_value="0.0",
        description="Camera Z offset from link5 (meters)",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": True},
        ],
        remappings=[("/joint_states", arm_joint_states_topic)],
        output="screen",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {"robot_description": robot_desc},
            controller_yaml,
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"robot_model_path": LaunchConfiguration("mjcf_path")},
            {"show_gui": LaunchConfiguration("show_gui")},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotic_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_arm_calc")),
    )

    # 真实摄像头 PnP 节点
    pnp_ros_node = Node(
        package="pnp_ros",
        executable="pnp_ros_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # 真实相机相对于link5的静态TF（根据实际安装位置调整）
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            LaunchConfiguration("camera_x"),
            LaunchConfiguration("camera_y"),
            LaunchConfiguration("camera_z"),
            "0", "0", "0",
            "link5", "camera_link"
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera_tf")),
    )

    # 相机光学坐标系静态TF（与真实机配置保持一致）
    camera_optical_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "-1.5708", "0", "-1.5708",
            "camera_link", "camera_optical_frame"
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera_tf")),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            {"robot_description": robot_desc},
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"robot_description": robot_desc},
            {"use_sim_time": True},
        ],
        remappings=[("/joint_states", arm_joint_states_topic)],
        condition=IfCondition(LaunchConfiguration("start_move_group")),
    )

    robotic_task = Node(
        package="robotic_task",
        executable="robotic_task",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            moveit_config.robot_description_semantic,
            {"use_sim_time": True},
        ],
        remappings=[("/joint_states", arm_joint_states_topic)],
        condition=IfCondition(LaunchConfiguration("start_robotic_task")),
    )

    load_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo started, spawning controllers"),
                joint_state_broadcaster,
                arm_controller,
            ],
        )
    )

    return LaunchDescription([
        mjcf_path_arg,
        show_rviz_arg,
        show_gui_arg,
        start_arm_calc_arg,
        start_move_group_arg,
        start_robotic_task_arg,
        start_camera_arg,
        start_camera_tf_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        robot_state_pub,
        mujoco,
        load_controller,
        camera_static_tf,
        camera_optical_static_tf,
        move_group,
        robotic_task,
        pnp_ros_node,
        arm_calc,
        rviz2,
    ])
