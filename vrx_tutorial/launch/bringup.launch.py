import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vrx_gz_dir = get_package_share_directory('vrx_gz')
    vrx_tutorial_dir = get_package_share_directory('vrx_tutorial')

    default_rviz_config = os.path.join(vrx_tutorial_dir, 'rviz', 'tutorial.rviz')

    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_delay = LaunchConfiguration('nav2_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')

    # 1. Gazebo + WAM-V + ros_gz bridges
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_gz_dir, 'launch', 'competition.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # 2. Nav2 (delayed so the Gazebo clock is up before nav2 lifecycle starts)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_tutorial_dir, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    nav2_delayed = TimerAction(period=nav2_delay, actions=[nav2_launch])

    # 3. RViz2 with the tutorial configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    rviz_delayed = TimerAction(period=rviz_delay, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Gazebo world to load'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Absolute path to the RViz2 config file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument(
            'nav2_delay',
            default_value='5.0',
            description='Seconds to wait before launching Nav2'),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='3.0',
            description='Seconds to wait before launching RViz2'),
        gazebo_launch,
        nav2_delayed,
        rviz_delayed,
    ])
