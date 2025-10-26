from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup_nodes(context):
    urdf_path = Path(LaunchConfiguration("urdf_path").perform(context)).expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"未找到指定的 URDF 文件: {urdf_path}")

    rviz_path_cfg = LaunchConfiguration("rviz_config").perform(context)
    rviz_args = ["-d", rviz_path_cfg] if rviz_path_cfg else []

    with urdf_path.open("r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/rviz_joint_states")],
            output="screen",
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            remappings=[("joint_states", "/rviz_joint_states")],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=rviz_args,
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                # 若想在文件内默认使用你的 URDF，请把下面替换为你的绝对路径，例如：
                default_value="/home/ma/桌面/miniArm_only_gazebo/miniArm_URDF/urdf/miniarm_rviz.urdf",
                description="URDF 文件绝对路径",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="可选的 RViz 配置文件路径（留空使用默认界面）",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时间",
            ),
            OpaqueFunction(function=_setup_nodes),
        ]
    )