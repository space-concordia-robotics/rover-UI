from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='sc_rover_sim', executable='camera_sim', output='screen'),
        Node(package='sc_rover_sim', executable='gps_sim', output='screen'),
    ])
