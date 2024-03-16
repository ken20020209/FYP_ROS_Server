# cartographer.launch.py
 
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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    basic_dir = get_package_share_directory('basic')
    cartographer_prefix = get_package_share_directory('basic')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='xgo_2d_loc.lua')
    #configuration_basename  = '/home/parallels/cartoros2/src/cartographer_ros/cartographer_ros/configuration_files/xgo_2d_loczition.lua'
    load_state_filename = LaunchConfiguration('load_state_filename', default=os.path.join(basic_dir, 'map', 'cartographer','mymap.pbstream'))
    # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    # print(load_state_filename)
    # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")

    map=LaunchConfiguration('map', default=os.path.join(basic_dir, 'map', 'cartographer', 'mymap.yaml'))

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    localizer_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py']
    )
    namespace=LaunchConfiguration('namespace', default='')

    cartographer_config_dir_declare = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=cartographer_config_dir,
        description='Full path to config file to load')
    configuration_basename_declare = DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of lua file for cartographer')
    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    load_state_filename_declare = DeclareLaunchArgument(
        'load_state_filename',
        default_value=load_state_filename,
        description='')
    sim_declare = DeclareLaunchArgument(
        name='sim',
        default_value='False',
        description='Enable use_sime_time to true'
    )
    rviz_declare = DeclareLaunchArgument(
        name='rviz',
        default_value='True',
        description='Enable rviz'
    )
    map_declare = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(basic_dir, 'map', 'cartographer', 'mymap.yaml'),
        description='Full path to map file to load'
    )
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Namespace for the controller'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename,
                   '-load_state_filename', load_state_filename],
        remappings=[('/scan', '/scan'), ('/imu', '/imu/data')])
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
    navgation=Node(
        package='basic',
        executable='Navigation',
        name='NavigationServer',
        output='screen'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(basic_dir, 'rviz', 'nav2_default_view.rviz')],
    )

    group = GroupAction(actions=[
        PushRosNamespace(namespace),
        cartographer_node,
        nav2_launch,
        localizer_launch,
        rviz,
        navgation
    ])

    launch_description = LaunchDescription()

    launch_description.add_action(cartographer_config_dir_declare)
    launch_description.add_action(configuration_basename_declare)
    launch_description.add_action(use_sim_time_declare)
    launch_description.add_action(load_state_filename_declare)
    launch_description.add_action(sim_declare)
    launch_description.add_action(rviz_declare)
    launch_description.add_action(map_declare)
    launch_description.add_action(namespace_declare)

    launch_description.add_action(group)


    return launch_description
