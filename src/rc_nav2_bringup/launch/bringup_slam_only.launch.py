import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    rf2o_base_frame = LaunchConfiguration('rf2o_base_frame')
    rf2o_odom_frame = LaunchConfiguration('rf2o_odom_frame')
    rf2o_odom_topic = LaunchConfiguration('rf2o_odom_topic')
    rf2o_freq = LaunchConfiguration('rf2o_freq')
    use_rviz = LaunchConfiguration('use_rviz')
    start_rf2o = LaunchConfiguration('start_rf2o')
    carto_odom_topic = LaunchConfiguration('carto_odom_topic')

    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rf2o.launch.py')),
        condition=IfCondition(start_rf2o),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'scan_topic': scan_topic,
            'base_frame_id': rf2o_base_frame,
            'odom_frame_id': rf2o_odom_frame,
            'odom_topic': rf2o_odom_topic,
            'freq': rf2o_freq,
        }.items()
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup_cartographer.launch.py')),
        launch_arguments={
            'use_gazebo': 'False',
            'use_display': 'False',
            'use_rviz': use_rviz,
            'scan_topic': scan_topic,
            'odom_topic': carto_odom_topic,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/rc_car/scan',
            description='Laser scan topic'),
        DeclareLaunchArgument(
            'rf2o_base_frame',
            default_value='rc_car/base_link_rot',
            description='RF2O base_frame_id'),
        DeclareLaunchArgument(
            'rf2o_odom_frame',
            default_value='rc_car/odom_rf2o',
            description='RF2O odom_frame_id'),
        DeclareLaunchArgument(
            'rf2o_odom_topic',
            default_value='/rc_car/odom_rf2o',
            description='RF2O odom topic name'),
        DeclareLaunchArgument(
            'carto_odom_topic',
            default_value='/rc_car/odom_rf2o_rewritten',
            description='Cartographer odom topic name'),
        DeclareLaunchArgument(
            'rf2o_freq',
            default_value='10.0',
            description='RF2O 처리 주기 (Hz)'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Whether to launch RViz (map can be viewed in existing RViz)'),
        DeclareLaunchArgument(
            'start_rf2o',
            default_value='True',
            description='RF2O를 이 런치에서 켤지 여부 (Gazebo에서 이미 구동 중이면 False)'),
        rf2o_launch,
        cartographer_launch,
    ])
