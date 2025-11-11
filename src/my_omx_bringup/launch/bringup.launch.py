from setuptools import find_packages
import controller_manager
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    description_pkg = FindPackageShare("my_omx_description").find("my_omx_description")
    bringup_pkg = FindPackageShare("my_omx_bringup").find("my_omx_bringup")

    # URDF 로드
    xacro_path = f"{description_pkg}/urdf/main.urdf.xacro"
    doc = xacro.process_file(xacro_path)
    robot_description = doc.toxml()

    controller_config = PathJoinSubstitution([bringup_pkg, "config", "controller.yaml"])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )
