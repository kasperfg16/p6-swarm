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

MAP_NAME = 'swarm_map1'  # change to the name of your own map here


def generate_launch_description():

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('process_robot_gazebo'),
         'launch', 'gazebo.launch.py']
    )
    
    nav2_config_path = PathJoinSubstitution(
            [FindPackageShare('process_robot_navigation'),
            'config', 'navigation_gazebo.yaml']
        )
    
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("process_robot_base"), "config", "ekf.yaml"]
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('process_robot_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('process_robot_navigation'), 'config', 'slam.yaml']
    )

    default_nav_to_pose_bt_xml_path = PathJoinSubstitution(
            [FindPackageShare('process_robot_navigation'), 'config', 'bt_config.xml']
        )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'params_file'
    if ros_distro == 'galactic': 
        slam_param_name = 'slam_params_file'

    lifecycle_nodes = [
        'urdf_spawner',
        'joint_state_publisher',
        'robot_state_publisher',
        'rviz2',
        'ekf_filter_node',
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower'
        ]
    
    return LaunchDescription([

        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='default_nav_to_pose_bt_xml',
            default_value=default_nav_to_pose_bt_xml_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='True',
            description='Use simulation time'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
        ),

        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch_path),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration("use_sim_time"),
                        slam_param_name: slam_config_path
                    }.items(),
                ),
        
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch_path),
                    launch_arguments={
                        'map': LaunchConfiguration("map"),
                        'use_sim_time': LaunchConfiguration("use_sim_time"),
                        'params_file': nav2_config_path,
                    }.items(),
                ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        )
    ])