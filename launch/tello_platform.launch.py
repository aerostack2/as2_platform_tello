#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Launch DJI Tello platform node."""

__authors__ = 'Daniel Fernández Sánchez'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Entrypoint."""
    control_modes = PathJoinSubstitution(
        [FindPackageShare('as2_platform_tello'),
         'config', 'control_modes.yaml']
    )

    platform_config_file = PathJoinSubstitution(
        [FindPackageShare('as2_platform_tello'), 'config',
         'platform_default.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value=EnvironmentVariable(
                    'AEROSTACK2_SIMULATION_DRONE_ID'),
                description='Drone namespace',
            ),
            DeclareLaunchArgument(
                'control_modes_file',
                default_value=control_modes,
                description='Platform control modes file',
            ),
            DeclareLaunchArgument(
                'platform_config_file',
                default_value=platform_config_file,
                description='Platform configuration file',
            ),
            Node(
                package='as2_platform_tello',
                executable='as2_platform_tello_node',
                name='platform',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                emulate_tty=True,
                parameters=[
                    {
                        'control_modes_file': LaunchConfiguration('control_modes_file'),
                    },
                    LaunchConfiguration('platform_config_file'),
                ],
            ),
        ]
    )
