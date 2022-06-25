# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import TimerAction

MAP_NAME = 'swarm_group_room_map'  # change to the name of your own map here

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    map = LaunchConfiguration("map")
    autostart = LaunchConfiguration('autostart')
    
    nav2_config_path = PathJoinSubstitution(
            [FindPackageShare('process_robot_navigation'),
            'config', 'navigation_real_robot.yaml']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('process_robot_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    
    return LaunchDescription([

        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='False',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='namespace', 
            default_value='',
            description='Define namespace'
        ),
        
        DeclareLaunchArgument(
            name='autostart', 
            default_value='True',
            description='Automatically startup the nav2 stack'
        ),
            
        # https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html#initialize-the-location-of-turtlebot-3 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'map': map,
                'use_sim_time': use_sim_time,
                'params_file': nav2_config_path,
                'autostart': autostart,
                'namespace': LaunchConfiguration('namespace')
            }.items(),
        )
    ])