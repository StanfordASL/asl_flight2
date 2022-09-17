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
    rc_mode_launch_arg = DeclareLaunchArgument("rc_mode", default_value="velocity",
                                               description="different RC control mode",
                                               choices=["velocity", "attitude", "body_rate"])

    return LaunchDescription([
        launch_rviz_launch_arg,
        vrpn_name_launch_arg,
        vrpn_server_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("asl_flight2"),
                    "launch",
                    "rviz.launch.py",
                ])
            ),
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("asl_flight2"),
                    "launch",
                    "hardware_base.launch.py",
                ])
            ]),
            launch_arguments={
                "vrpn_name": LaunchConfiguration("vrpn_name"),
                "vrpn_server": LaunchConfiguration("vrpn_server"),
            }.item()
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy",
        ),
        # wait for px4_agenet
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="asl_flight2",
                    namespace="asl",
                    executable="controller_ps4",
                    output="screen",
                    parameters=[{
                        "mode": LaunchConfiguration("rc_mode"),
                    }],
                ),
            ],
        ),
    ])

