# cartographer.launch.py
 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node,PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.conditions import IfCondition
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_prefix = get_package_share_directory('basic')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='xgo_2d.lua')
 
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    rviz=LaunchConfiguration('rviz', default='True')
    namespace = LaunchConfiguration('namespace', default='')

    cartographer_config_dir_declare=DeclareLaunchArgument(
                                    'cartographer_config_dir',
                                    default_value=cartographer_config_dir,
                                    description='Full path to config file to load')
    
    configuration_basename_declare=DeclareLaunchArgument(
                                    'configuration_basename',
                                    default_value=configuration_basename,
                                    description='Name of lua file for cartographer')
    

    use_sim_time_declare=DeclareLaunchArgument(
                                    'use_sim_time',
                                    default_value=use_sim_time,
                                    description='Use simulation (Gazebo) clock if true')
    rviz_declare=DeclareLaunchArgument(
                                    'rviz',
                                    default_value='True',
                                    description='Enable rviz')
    namespace_declare=DeclareLaunchArgument(    
                                    'namespace',
                                    default_value='',
                                    description='Namespace for the controller')
    
    resolution_declare=DeclareLaunchArgument(
                                    'resolution',
                                    default_value=resolution,
                                    description='Resolution of a grid cell in the published occupancy grid')
 
    publish_period_sec_declare=DeclareLaunchArgument(
                                    'publish_period_sec',
                                    default_value=publish_period_sec,
                                    description='OccupancyGrid publishing period')
    
    cartographer_node= Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            #remappings=[('/scan', '/MS200/scan')]),
            remappings=[('/scan', '/scan'),('/imu', '/imu/data')])

    rviz_node=Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(rviz),
                output='screen')
    
    occupancy_grid_launch=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                                'publish_period_sec': publish_period_sec}.items())
    
    groupAction=GroupAction(
        actions=[
            PushRosNamespace(namespace),
            cartographer_node,
            occupancy_grid_launch,
            rviz_node
        ]
    )

    launchDescription=LaunchDescription()
    launchDescription.add_action(cartographer_config_dir_declare)
    launchDescription.add_action(configuration_basename_declare)
    launchDescription.add_action(use_sim_time_declare)
    launchDescription.add_action(rviz_declare)
    launchDescription.add_action(namespace_declare)
    launchDescription.add_action(resolution_declare)
    launchDescription.add_action(publish_period_sec_declare)

    launchDescription.add_action(groupAction)
    

    return launchDescription