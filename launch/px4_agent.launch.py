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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    sim_launch_arg = DeclareLaunchArgument('sim',
                                           default_value='true',
                                           description='use simulation (UDP connection)',
                                           choices=['false', 'true'])
    device_launch_arg = DeclareLaunchArgument('device',
                                              default_value='/dev/ttyS4',
                                              description='device for UART connection')
    baudrate_launch_arg = DeclareLaunchArgument('baudrate',
                                                default_value='921600',
                                                description='UART baudrate')

    return LaunchDescription([
        sim_launch_arg,
        device_launch_arg,
        baudrate_launch_arg,
        ExecuteProcess(
            cmd=[
                FindExecutable(name='micrortps_agent'),
                '-t', 'UDP',
            ],
            condition=IfCondition(LaunchConfiguration('sim')),
        ),
        ExecuteProcess(
            cmd=[
                FindExecutable(name='micrortps_agent'),
                '-t', 'UART',
                '-d', LaunchConfiguration('device'),
                '-b', LaunchConfiguration('baudrate'),
            ],
            condition=UnlessCondition(LaunchConfiguration('sim')),
        ),
    ])
