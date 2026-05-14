import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    vrx_tutorial_dir = get_package_share_directory('vrx_tutorial')
    params_file = os.path.join(vrx_tutorial_dir, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # Rewrite use_sim_time into params file
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # 1. Publish map -> odom static TF (identity since we use ground truth pose)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 1b. Fix frame name mismatch: Gazebo LaserScan uses 'wamv/base_link',
    # but robot_state_publisher outputs 'wamv/wamv/base_link' due to double prefix.
    static_base_link_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'wamv/wamv/base_link', 'wamv/base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. Bridge Gazebo world pose to ROS TF and /odom.
    gz_model_pose_bridge = Node(
        package='vrx_tutorial',
        executable='gz_model_pose_bridge',
        name='gz_model_pose_bridge',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Convert cmd_vel -> WAM-V thrusters
    cmd_vel_to_wamv = Node(
        package='vrx_tutorial',
        executable='cmd_vel_to_wamv',
        name='cmd_vel_to_wamv',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. RKP global planner (replaces Nav2 planner_server action)
    rkp_planner = Node(
        package='vrx_tutorial',
        executable='rkp_planner',
        name='rkp_planner',
        namespace='rkp',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 5. Nav2 navigation stack (custom launch that remaps bt_navigator's
    # ComputePathToPose action client to /rkp/ComputePathToPose so it talks
    # to rkp_planner instead of the default planner_server.)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_tutorial_dir, 'launch', 'navigation_launch_rkp.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        static_map_to_odom,
        static_base_link_fix,
        gz_model_pose_bridge,
        cmd_vel_to_wamv,
        rkp_planner,
        navigation_launch,
    ])
