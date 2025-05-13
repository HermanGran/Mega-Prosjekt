from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Definer argumenter
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='Kameraindeks for Video'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='5.0',
            description='Antall bilder per sekund'
        ),

        # Start camera_driver_node
        Node(
            package='camera_interface',
            executable='camera_driver_node',
            name='camera_driver_node',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'fps': LaunchConfiguration('fps'),
            }],
            output='screen'
        ),
    ])
