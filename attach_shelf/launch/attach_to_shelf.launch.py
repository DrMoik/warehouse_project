from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    

    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server'
    )

    return LaunchDescription([
    
        approach_service_server_node
    ])
