#!/usr/bin/python3.6
#
# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os
import pathlib

from launch import LaunchDescription
import launch.actions
from launch.substitutions import EnvironmentVariable
import launch_ros.actions


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'test_laser_assembler.yaml'
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    os.environ['TOPIC_NAME'] = 'scan'
    topic_prefix = 'dummy_'

    dummy_scan_producer = launch_ros.actions.Node(
        package='laser_assembler', node_executable='dummy_scan_producer', output='screen')

    laser_scan_assembler = launch_ros.actions.Node(
        package='laser_assembler', node_executable='laser_scan_assembler',
        output='screen',
        remappings=[
            ('scan', 'dummy_scan'),
            (EnvironmentVariable(name='TOPIC_NAME'), [
                topic_prefix, EnvironmentVariable(name='TOPIC_NAME')])
        ],
        parameters=[
            parameters_file_path,
            str(parameters_file_path),
            [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_laser_assembler.yaml'],
        ])

    test_laser_assembler = launch_ros.actions.Node(
        package='laser_assembler', node_executable='test_assembler',
        output='screen'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        dummy_scan_producer,
        laser_scan_assembler,
        test_laser_assembler,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=test_laser_assembler,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
