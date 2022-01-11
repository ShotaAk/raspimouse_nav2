# Copyright 2021 ShotaAk
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    raspimouse_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('raspimouse_description'),
            '/launch/display.launch.py']),
        launch_arguments={'lidar': 'urg',
                          'lidar_frame': 'lidar_link',
                          'use_rviz': 'false',
                          'gazebo': 'true',
                          }.items(),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'raspimouse',
                                   '-x', '0', '-y', '0', '-z', '0.3',
                                   '-topic', '/robot_description'],
                        output='screen')

    joint_state_controller = ExecuteProcess(
      cmd=['ros2 run controller_manager spawner.py joint_state_controller'],
      shell=True,
      output='screen'
    )

    diff_drive_controller = ExecuteProcess(
      cmd=['ros2 run controller_manager spawner.py diff_drive_controller'],
      shell=True,
      output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[diff_drive_controller],
            )
        ),
        raspimouse_model,
        spawn_entity,
    ])
