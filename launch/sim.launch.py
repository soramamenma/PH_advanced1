from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        SetEnvironmentVariable("GZ_PARTITION", "pai_navigation"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", str(root / "worlds" / "navigation.sdf")], output="screen"),
        ExecuteProcess(cmd=["gz", "sim", "-g"], condition=IfCondition(LaunchConfiguration("gui")), output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="navigation_bridge",
             parameters=[{"config_file": str(root / "config" / "bridge.yaml"), "use_sim_time": True}], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="lidar_frame",
             arguments=["--x", "0", "--y", "0", "--z", "0.32", "--roll", "0", "--pitch", "0", "--yaw", "0",
                        "--frame-id", "navbot/base_link", "--child-frame-id", "navbot/lidar_link"],
             parameters=[{"use_sim_time": True}], output="screen"),
    ])
