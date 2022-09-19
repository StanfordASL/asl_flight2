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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vrpn_name_launch_arg = DeclareLaunchArgument('vrpn_name', default_value='asl_drone',
                                                 description='name of the drone in motion capture')
    vrpn_server_launch_arg = DeclareLaunchArgument('vrpn_server', default_value='localhost',
                                                   description='vrpn server address IP')
    return LaunchDescription([
        vrpn_name_launch_arg,
        vrpn_server_launch_arg,
        Node(
            package='vrpn_mocap',
            executable='client_node',
            parameters=[{
                'server': LaunchConfiguration('vrpn_server'),
                'port': 3883,
                'frame_id': 'world_nwu',
                'update_freq': 100.,
                'refresh_freq': 1.,
                'multi_sensor': False,
            }],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1', '0', '0', '0', 'world_ned', 'world_nwu'],
        ),
        Node(
            package='asl_flight2',
            namespace='asl',
            executable='mocap_relay_vrpn',
            output='screen',
            parameters=[{
                'vrpn_name': LaunchConfiguration('vrpn_name'),
            }],
        ),
    ])
