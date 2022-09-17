from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import FindExecutable, LaunchConfiguration

def generate_launch_description():
    sim_launch_arg = DeclareLaunchArgument("sim",
        default_value="true",
        description="use simulation (UDP connection)",
        choices=["false", "true"])
    device_launch_arg = DeclareLaunchArgument("device",
        default_value="/dev/ttyS4",
        description="device for UART connection")
    baudrate_launch_arg = DeclareLaunchArgument("baudrate",
        default_value="921600",
        description="UART baudrate")

    return LaunchDescription([
        sim_launch_arg,
        device_launch_arg,
        baudrate_launch_arg,
        ExecuteProcess(
            cmd=[
                FindExecutable(name="micrortps_agent"),
                "-t", "UDP",
            ],
            condition=IfCondition(LaunchConfiguration("sim")),
        ),
        ExecuteProcess(
            cmd=[
                FindExecutable(name="micrortps_agent"),
                "-t", "UART",
                "-d", LaunchConfiguration("device"),
                "-b", LaunchConfiguration("baudrate"),
            ],
            condition=UnlessCondition(LaunchConfiguration("sim")),
        ),
    ])
