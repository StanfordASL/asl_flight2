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
    model_name_launch_arg = DeclareLaunchArgument('model_name', default_value='iris',
                                                  description='name of the drone in gazebo')

    return LaunchDescription([
        model_name_launch_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1', '0', '0', '0', 'world_ned', 'world_nwu'],
        ),
        Node(
            package='asl_flight2',
            # namespace=LaunchConfiguration('model_name'),
            executable='mocap_relay_sim',
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
            }],
        ),
    ])
