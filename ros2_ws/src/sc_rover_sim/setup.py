from setuptools import setup
package_name = 'sc_rover_sim'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_sensors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sc',
    maintainer_email='you@example.com',
    description='Simulated rover sensors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_sim = sc_rover_sim.camera_sim:main',
            'map_sim = sc_rover_sim.map_sim:main',
            'gps_sim = sc_rover_sim.gps_sim:main',
            'laserscan_sim = sc_rover_sim.laserscan_sim:main',
            'aruco_sim = sc_rover_sim.aruco_sim:main',
        ],
    },
)
