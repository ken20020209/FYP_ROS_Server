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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node,PushRosNamespace


def generate_launch_description():

    basic_dir = get_package_share_directory('basic')

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    localizer_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py']
    )
    
    rf2o_launch_tf_path = PathJoinSubstitution(
        [FindPackageShare('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry_tf.launch.py']
    )
    rf2o_launch_no_tf_path = PathJoinSubstitution(
        [FindPackageShare('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry_no_tf.launch.py']
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ekf_node_start = LaunchConfiguration('ekf_node', default='False')
    rviz = LaunchConfiguration('rviz', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    map = LaunchConfiguration('map', default=os.path.join(basic_dir, 'map','slam_toolbox', 'map.yaml'))

    use_sim_time_declare=DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Enable use_sime_time to true'
    )
    ekf_node_declare=DeclareLaunchArgument(
        'ekf_node',
        default_value='False',
        description='Enable ekf_node'
    )
    rviz_declare=DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Enable rviz'
    )
    namespace_declare=DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the controller'
    )
    map_declare=DeclareLaunchArgument(
        'map',
        default_value=os.path.join(basic_dir, 'map','slam_toolbox', 'map.yaml'),
        description='Full path to map file to load'
    )

    rf2o_launch_no_tf=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rf2o_launch_no_tf_path),
        condition=IfCondition(LaunchConfiguration('ekf_node'))
    )
    rf2o_launch_tf=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rf2o_launch_tf_path),
        condition=IfCondition(PythonExpression(['not ',LaunchConfiguration('ekf_node')])),
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('ekf_node')),
        parameters=[os.path.join(basic_dir, 'config','ekf.yaml'), {'use_sim_time': LaunchConfiguration('sim')}],
        remappings=[('/odometry/filtered','/odom')]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("sim"),
            'params_file': os.path.join(basic_dir, 'config', 'nav2_params_no_amcl_tf.yaml'),
        }.items()
    )
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localizer_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("sim"),
            'params_file': os.path.join(basic_dir, 'config', 'nav2_params_no_amcl_tf.yaml'),
            'map': map,
        }.items()
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(basic_dir, 'rviz', ' nav2_default_view.rviz')],
    )

    group = GroupAction(actions=[
        PushRosNamespace(namespace),
        rf2o_launch_no_tf,
        rf2o_launch_tf,
        robot_localization_node,
        nav2_launch,
        localizer_launch,
        rviz
    ])

    launch_description = LaunchDescription()

    launch_description.add_action(use_sim_time_declare)
    launch_description.add_action(ekf_node_declare)
    launch_description.add_action(rviz_declare)
    launch_description.add_action(namespace_declare)
    launch_description.add_action(map_declare)

    launch_description.add_action(group)

    return launch_description