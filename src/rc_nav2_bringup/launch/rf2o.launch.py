from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    odom_topic = LaunchConfiguration('odom_topic')
    freq = LaunchConfiguration('freq')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Laser scan topic'),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_link_rot',
            description='Base frame ID (TF parent of laser frame)'),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Odom frame ID'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom_rf2o',
            description='Published odom topic name'),
        DeclareLaunchArgument(
            'freq',
            default_value='5.0',
            description='RF2O 처리 주기 (Hz)'),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'laser_scan_topic': scan_topic,
                'odom_topic': odom_topic,
                'publish_tf': True,
                'base_frame_id': base_frame_id,
                'odom_frame_id': odom_frame_id,
                'init_pose_from_topic': '',
                'freq': freq
            }],
        ),
    ])
