from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([  
        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='Port for the rosbridge_websocket'
        ),
        Node(
            package='basic',
            namespace='',
            executable='RobotDogConnector'
        ),
        Node(
            package='rosbridge_server',
            namespace='',
            executable='rosbridge_websocket',
            parameters=[{'port': LaunchConfiguration('port')}]
        )
    ])