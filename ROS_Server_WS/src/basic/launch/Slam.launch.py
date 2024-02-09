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
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    basic_dir = get_package_share_directory('basic')

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )
    
    rf2o_launch_tf_path = PathJoinSubstitution(
        [FindPackageShare('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry_tf.launch.py']
    )
    rf2o_launch_no_tf_path = PathJoinSubstitution(
        [FindPackageShare('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry_no_tf.launch.py']
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='False',
            description='Enable use_sime_time to true'
        ),
        DeclareLaunchArgument(
            name='ekf_node',
            default_value='False',
            description='Enable ekf_node'
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rf2o_launch_no_tf_path),
            condition=IfCondition(LaunchConfiguration('ekf_node'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rf2o_launch_tf_path),
            condition=IfCondition(PythonExpression(['not ',LaunchConfiguration('ekf_node')])),
        )
        ,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('ekf_node')),
            parameters=[os.path.join(basic_dir, 'config','ekf.yaml'), {'use_sim_time': LaunchConfiguration('sim')}],
            remappings=[('/odometry/filtered','/odom')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': os.path.join(basic_dir, 'config', 'nav2_params.yaml')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'slam_params_file': os.path.join(basic_dir, 'config', 'mapper_params_online_async.yaml')
            }.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(basic_dir, 'rviz', ' nav2_default_view.rviz')],
        )
    ])