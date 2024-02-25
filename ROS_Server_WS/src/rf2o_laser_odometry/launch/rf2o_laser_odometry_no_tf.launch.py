import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    namespace=LaunchConfiguration('namespace',default='')
    namespace_declare=DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Name of the RobotDogConnector node'
        )
    return LaunchDescription([
            namespace_declare,
            DeclareLaunchArgument(
                name='publish_tf',
                default_value='False',
                description='Publish the odometry transform'
            ),
            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : 'scan',
                    'odom_topic' : 'odom',
                    'publish_tf' : LaunchConfiguration('publish_tf'),
                    'base_frame_id' : PythonExpression(["'",namespace,"'+","'base_footprint'"]),
                    'odom_frame_id' : PythonExpression(["'",namespace,"'+","'odom'"]),
                    'init_pose_from_topic' : '',
                    'freq' : 5.0}],
            ),
    ])