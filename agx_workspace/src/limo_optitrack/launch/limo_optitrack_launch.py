from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limo_optitrack',
            executable='limo_pos_publisher',
            name='limo_pos_publisher',
            output='screen',
        ),
    ])
