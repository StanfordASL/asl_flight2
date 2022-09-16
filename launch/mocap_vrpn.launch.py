from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_rviz_launch_arg = DeclareLaunchArgument("launch_rviz", default_value="false",
                                                   description="set to true to launch rviz",
                                                   choices=["false", "true"])
    vrpn_name_launch_arg = DeclareLaunchArgument("vrpn_name", default_value="asl_drone",
                                                 description="name of the drone in motion capture")
    vrpn_server_launch_arg = DeclareLaunchArgument("vrpn_server", default_value="localhost",
                                                   description="vrpn server address IP")
    return LaunchDescription([
        launch_rviz_launch_arg,
        vrpn_name_launch_arg,
        vrpn_server_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("asl_flight2"),
                    "launch",
                    "rviz.py",
                ])
            ),
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
        Node(
            package="vrpn_mocap",
            executable="client_node",
            parameters=[{
                "server": LaunchConfiguration("vrpn_server"),
                "port": 3883,
                "frame_id": "world_nwu",
                "update_freq": 100.,
                "refresh_freq": 1.,
                "multi_sensor": False,
            }],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = ["0", "0", "0", "1", "0", "0", "0", "world_ned", "world_nwu"],
        ),
        Node(
            package="asl_flight2",
            namespace="asl",
            executable="mocap_relay_vrpn",
            output="screen",
            parameters=[{
                "vrpn_name": LaunchConfiguration("vrpn_name"),
            }],
        ),
    ])

