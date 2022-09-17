from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("asl_flight2"),
                    "launch",
                    "px4_agent.launch.py",
                ])
            ),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = ["0", "0", "0", "1", "0", "0", "0", "world_ned", "world_nwu"],
        ),
        # wait for px4_agenet
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="asl_flight2",
                    namespace="asl",
                    executable="mocap_relay_sim",
                    output="screen",
                ),
            ],
        ),
    ])
