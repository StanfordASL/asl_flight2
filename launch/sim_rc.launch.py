# Copyright 2022 Stanford ASL
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_rviz_launch_arg = DeclareLaunchArgument('launch_rviz', default_value='false',
                                                   description='set to true to launch rviz',
                                                   choices=['false', 'true'])
    rc_mode_launch_arg = DeclareLaunchArgument('rc_mode', default_value='velocity',
                                               description='different RC control mode',
                                               choices=['velocity', 'attitude', 'body_rate'])
    asl_flight2_share = FindPackageShare('asl_flight2')

    return LaunchDescription([
        launch_rviz_launch_arg,
        rc_mode_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    asl_flight2_share,
                    'launch',
                    'rviz.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    asl_flight2_share,
                    'launch',
                    'sim_base.launch.py',
                ])
            )
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
        ),
        # wait for px4_agenet
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='asl_flight2',
                    namespace='asl',
                    executable='controller_ps4',
                    output='screen',
                    parameters=[{
                        'mode': LaunchConfiguration('rc_mode'),
                    }],
                ),
            ],
        ),
    ])
