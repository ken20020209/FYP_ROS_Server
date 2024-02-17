from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([  
        Node(
            package='basic',
            namespace='',
            executable='Camera'
        ),
    ])