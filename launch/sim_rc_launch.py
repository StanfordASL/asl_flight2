from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_launch_arg = DeclareLaunchArgument("launch_rviz", default_value="false",
                                                   description="set to true to launch rviz",
                                                   choices=["false", "true"])
    return LaunchDescription([
        launch_rviz_launch_arg,
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(launch_rviz),
            arguments = ["-d", PathJoinSubstitution([FindPackageShare("asl_flight2"), "rviz", "default.rviz"])]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = ["0", "0", "0", "1", "0", "0", "0", "world_ned", "world_nwu"],
        ),
        Node(
            package="asl_flight2",
            namespace="asl",
            executable="mocap_relay_sim",
            output="screen",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy",
        ),
        Node(
            package="asl_flight2",
            namespace="asl",
            executable="controller_ps4",
            output="screen",
        ),
    ])
