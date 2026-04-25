from setuptools import setup

package_name = 'vrx_tutorial'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Tutorial nodes for VRX simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = vrx_tutorial.obstacle_avoidance:main',
            'keyboard_teleop = vrx_tutorial.keyboard_teleop:main',
            'cmd_vel_to_wamv = vrx_tutorial.cmd_vel_to_wamv:main',
        ],
    },
)
