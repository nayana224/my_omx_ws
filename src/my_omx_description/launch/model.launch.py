from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    xacro_path = "/home/pyo/Documents/ros2_ws/my_omx_ws/src/my_omx_description/urdf/main.urdf.xacro"
    doc = xacro.process_file(xacro_path)
    robot_description = doc.toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz2 = Node(package="rviz2", executable="rviz2", name="rviz2")

    return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz2])
