from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():


#获取其他launch文件的目录
    moveit_config_dir = get_package_share_directory("robotic_config")
    launch_dir = os.path.join(moveit_config_dir, "launch")

    #用于驱动真实机器人的节点
    joint_driver=Node(
            package="robot_driver",
            executable="robot_driver",
            output="screen",
            parameters=[{
                "use_sim_time": False
            }]
        )
    
    #启动前馈力矩计算节点
    joint_dynamic_calc=Node(
            package="my_controller",
            executable="my_controller",
            output="screen",
            parameters=[{
                "use_sim_time": False
            }]
        )

    #选择启动Moveit Setup Assistant生成的launch文件
    robot_description_launch=os.path.join(launch_dir,"rsp.launch.py")
    move_group_launch = os.path.join(launch_dir, "move_group.launch.py")
    rviz_launch        = os.path.join(launch_dir, "moveit_rviz.launch.py")
    ros_control_launch        = os.path.join(launch_dir, "bringup.launch.py")

    start_ros_control=IncludeLaunchDescription(PythonLaunchDescriptionSource(ros_control_launch))

    robot_description_launch_py=IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_description_launch))
    
    move_group=IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch))

    rviz_show=IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz_launch))


    return LaunchDescription([robot_description_launch_py,move_group,rviz_show,joint_driver,joint_dynamic_calc,start_ros_control])
