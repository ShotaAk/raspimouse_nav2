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
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('raspimouse_nav2'),
            '/launch/gazebo_field.launch.py']),
    )
    spawn_raspimouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('raspimouse_nav2'),
            '/launch/spawn_raspimouse.launch.py']),
    )
    spawn_simple_mobile_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('raspimouse_nav2'),
            '/launch/spawn_simple_mobile_robot.launch.py']),
    )
    rviz_config_file = get_package_share_directory(
        'raspimouse_nav2') + '/launch/slam.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file]
    )

    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory(
                'raspimouse_nav2')
            + '/config/mapper_params_offline.yaml'
        ],
    )

    timer_spawn_raspimouse = TimerAction(
        period=5.0,
        actions=[
            spawn_raspimouse,
        ]
    )

    timer_spawn_simple_mobile_robot = TimerAction(
        period=5.0,
        actions=[
            spawn_simple_mobile_robot,
        ]
    )


    timer_start_rviz = TimerAction(
        period=10.0,
        actions=[
            rviz_node
        ]
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo)
    # ld.add_action(timer_spawn_raspimouse)
    ld.add_action(timer_spawn_simple_mobile_robot)
    ld.add_action(timer_start_rviz)
    # ld.add_action(slam_node)

    return ld
