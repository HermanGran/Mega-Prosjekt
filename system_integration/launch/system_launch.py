from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Argumenter for kamera
        DeclareLaunchArgument(
            'camera_index',
            default_value='2',
            description='Kameraindeks for Video'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30.0',
            description='Bilder per sekund fra kameraet'
        ),

        # Start task_manager_node
#        Node(
#            package='system_integration',
#            executable='task_manager_node',
#            name='task_manager',
#            output='screen'
#        ),

        # Start camera_driver_node
        Node(
            package='camera_interface',
            executable='camera_driver_node',
            name='camera_driver_node',
            output='screen',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'fps': LaunchConfiguration('fps')
            }]
        ),

        # Start cube_detector_node
        Node(
            package='cube_detection',
            executable='cube_detector_node',
            name='cube_detector',
            output='screen'
        ),

        # Start pose_estimator_node
        Node(
            package='cube_detection',
            executable='pose_estimator_node',
            name='pose_estimator_node',
            output='screen'
        )
    ])
