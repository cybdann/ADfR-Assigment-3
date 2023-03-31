from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='cam_capture',
            executable='light_position_indicator',
            name='light_position_indicator'
        ),
        
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
        ),

        Node(
            package='image_tools',
            executable='showimage',
            name='showimage',
        ),

        # Node(
        #     package='cam_capture',
        #     executable='brightness_level_detector',
        #     name='brightness_level_detector'
        # ),

        Node(
            package='closed_loop',
            executable='controller',
            name='controller'
        ),

        Node(
            package='jiwy_simulator',
            executable='jiwy_simulator',
            name='jiwy_simulator'
        ),

        Node(
            package='image_tools',
            executable='showimage',
            remappings=[('image', '/moving_camera_output')],
            name='showimage2'
        )
    ])
