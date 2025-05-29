from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Sjekk at RViz-konfigurasjonen eksisterer
    rviz_config_path = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'rviz',
        'ur_moveit.rviz'  # Endre filnavn om nødvendig
    )

    # Konfigurasjonsfil for motion planner
    config_path = os.path.join(
        os.getenv('HOME'),
        'mega_ws/src/ur_motion_planning/config/poses.yaml'
    )

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

        # Start RViz med MoveIt-konfigurasjon
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # Static transform fra base_link til kamera_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0.0', '0.0', '-0.05',
                       '0.0', '0.0', '0.0',
                       'base_link', 'camera_frame'],
            output='screen'
        ),

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
        ),

        # Start motion_planner_node
        Node(
            package='ur_motion_planning',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[config_path]  # Denne må være en liste med strenger eller dicts
        ),

        # Delay-start task_manager_node for å vente på andre noder
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='system_integration',
                    executable='task_manager_node',
                    name='task_manager',
                    output='screen'
                )
            ]
        ),
    ])
