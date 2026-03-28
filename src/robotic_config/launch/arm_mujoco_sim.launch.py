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
        robot_state_pub,
        mujoco,
        load_controller,
        move_group,
        arm_calc,
        rviz2,
    ])
