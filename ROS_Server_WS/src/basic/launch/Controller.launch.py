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
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the controller'
        ),
        Node(
            package='basic',
            namespace=LaunchConfiguration('namespace'),
            executable='Controller',
        ),
        # start rosbrige server
        Node(
            package='rosbridge_server',
            namespace=LaunchConfiguration('namespace'),
            executable='rosbridge_websocket',
            parameters=[
                {'port': LaunchConfiguration('port'),
                 'call_services_in_new_thread': False,
                 'send_action_goals_in_new_thread':False
            }]
        ),
        Node(
            package='rosapi',
            namespace=LaunchConfiguration('namespace'),
            executable='rosapi_node'
        ),
            
        Node(
            package='basic',
            # namespace=LaunchConfiguration('namespace'),
            executable='Camera',
            output='screen'

        )
    ])