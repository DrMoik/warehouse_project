import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map file to load')

    map_file = LaunchConfiguration('map_file')

    # Use PathJoinSubstitution to create a valid path
    yaml_file_path = PathJoinSubstitution([
        get_package_share_directory('map_server'), 'config', map_file
    ])
    
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('localization_server'), 'config', 'localizer_rviz_config.rviz')

     # Check if the map_file_arg is equal to the default value 'warehouse_map_sim.yaml'
    is_default_map = map_file == 'warehouse_map_sim.yaml'

    return LaunchDescription([
        # Include the declaration of the launch argument
        map_file_arg,
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ) ,


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': is_default_map}, 
                        {'yaml_filename':yaml_file_path} 
                       ]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': is_default_map},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )      

               
    ])
