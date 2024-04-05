from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([  
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the controller'
        ),
        DeclareLaunchArgument(
            'camera_url',
            default_value='http://localhost:80/effect',
            description='URL for camera effect'
        ),
        Node(
            package='basic',
            namespace=LaunchConfiguration('namespace'),
            executable='Camera',
            parameters=[{'url': LaunchConfiguration('camera_url')}]
        ),
    ])