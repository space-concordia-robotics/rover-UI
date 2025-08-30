from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
def generate_launch_description():
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(os.path.dirname(__file__), '../../sc_rover_sim/launch/sim_sensors.launch.py')]))
    return LaunchDescription([sim])
