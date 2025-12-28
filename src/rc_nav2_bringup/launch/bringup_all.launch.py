import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_nav2_bringup')
    pkg_desc = get_package_share_directory('rc_car_test_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_display = LaunchConfiguration('use_display')
    rf2o_base_frame = LaunchConfiguration('rf2o_base_frame')
    rf2o_odom_frame = LaunchConfiguration('rf2o_odom_frame')
    rf2o_odom_topic = LaunchConfiguration('rf2o_odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    rf2o_freq = LaunchConfiguration('rf2o_freq')
    carto_odom_topic = LaunchConfiguration('carto_odom_topic')

    # rf2o odometry (scan -> odom_rf2o TF)
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rf2o.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'scan_topic': scan_topic,
            'base_frame_id': rf2o_base_frame,
            'odom_frame_id': rf2o_odom_frame,
            'odom_topic': rf2o_odom_topic,
            'freq': rf2o_freq,
        }.items()
    )

    # robot_state_publisher (+optional GUI/RViz) to provide base_link TF
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'display.launch.py')
        ),
        condition=IfCondition(PythonExpression([use_display, ' and not ', use_gazebo])),
        launch_arguments={
            'gui': 'False',
            'use_rviz': 'False',
        }.items()
    )

    # Cartographer SLAM (map + map->odom TF)
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup_cartographer.launch.py')),
        launch_arguments={
            'use_gazebo': use_gazebo,
            'use_display': 'False',
            'scan_topic': scan_topic,
            'odom_topic': carto_odom_topic,
        }.items()
    )

    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='False',
            description='Gazebo 시뮬레이션을 켤지 여부'),
        DeclareLaunchArgument(
            'use_display',
            default_value='True',
            description='URDF robot_state_publisher 구동 여부'),
        DeclareLaunchArgument(
            'rf2o_base_frame',
            default_value='base_link',
            description='RF2O base_frame_id'),
        DeclareLaunchArgument(
            'rf2o_odom_frame',
            default_value='odom',
            description='RF2O odom_frame_id'),
        DeclareLaunchArgument(
            'rf2o_odom_topic',
            default_value='/odom_rf2o',
            description='RF2O odom topic name'),
        DeclareLaunchArgument(
            'carto_odom_topic',
            default_value='/rc_car/odom_rf2o_rewritten',
            description='Cartographer odom topic name'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Laser scan topic'),
        DeclareLaunchArgument(
            'rf2o_freq',
            default_value='5.0',
            description='RF2O 처리 주기 (Hz)'),
        display_launch,
        rf2o_launch,
        cartographer_launch,
        *nav2_nodes,
    ])