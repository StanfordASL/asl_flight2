from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments = [
                "-d",
                PathJoinSubstitution([FindPackageShare("asl_flight2"), "rviz", "default.rviz"]),
            ]
        )
    ])
