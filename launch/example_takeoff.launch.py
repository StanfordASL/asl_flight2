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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import \
    LaunchConfiguration, PathJoinSubstitution, PythonExpression, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    name_launch_arg = DeclareLaunchArgument('name', default_value='iris',
                                            description='name of the drone in motion capture')
    vrpn_server_launch_arg = DeclareLaunchArgument('vrpn_server', default_value='localhost',
                                                   description='vrpn server address IP')
    platform_launch_arg = DeclareLaunchArgument('platform', default_value='sim',
                                                description='run on sim or hardware',
                                                choices=['sim', 'hardware'])

    return LaunchDescription([
        name_launch_arg,
        vrpn_server_launch_arg,
        platform_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    PythonExpression([
                        '"', LaunchConfiguration('platform'), '"',
                        ' + "_base.launch.py"'
                    ]),
                ])
            ]),
            launch_arguments={
                'model_name': LaunchConfiguration('name'),
                'vrpn_name': LaunchConfiguration('name'),
                'vrpn_server': LaunchConfiguration('vrpn_server'),
            }.items()
        ),
        Node(
            package='asl_flight2',
            namespace=LaunchConfiguration('name'),
            executable='example_takeoff',
            output='screen',
        ),
    ])
