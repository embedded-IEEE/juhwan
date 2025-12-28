from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ns = LaunchConfiguration('ns')

    pkg_desc = get_package_share_directory('rc_car_test_description')

    controllers_yaml = PathJoinSubstitution(
        [pkg_desc, 'config', 'rc_car_controllers.yaml']
    )

    ros_gz_bridge_yaml = PathJoinSubstitution(
        [pkg_desc, 'config', 'ros_gz_bridge_gazebo.yaml']
    )

    scan_rewriter = Node(
        package='rc_car_test_description',
        executable='scan_frame_rewriter',
        name='scan_frame_rewriter',
        output='screen',
        parameters=[{'target_frame': 'rc_car/lidar_link_1_back', 'use_sim_time': True}],
        namespace=ns,
        remappings=[
            ('scan_in',  'scan_raw'),  # 입력: bridge에서 나온 원래 스캔
            ('scan_out', 'scan'),      # 출력: frame_id 고친 스캔
        ]
    )

    odom_rewriter = Node(
        package='rc_car_test_description',
        executable='odom_stamp_rewriter',
        name='odom_stamp_rewriter',
        output='screen',
        parameters=[{'use_sim_time': True, 'min_step_ms': 1.0}],
        namespace=ns,
        remappings=[
            ('odom_in', 'odom_rf2o'),
            ('odom_out', 'odom_rf2o_rewritten'),
        ]
    )


    js_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", ["/", ns, "/controller_manager"],
                   "--param-file", controllers_yaml],
        output="screen",
        namespace=ns
    )

    steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller",
                   "--controller-manager", ["/", ns, "/controller_manager"],
                   "--param-file", controllers_yaml],
        output="screen",
        namespace=ns
    )

    wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_controller",
                   "--controller-manager", ["/", ns, "/controller_manager"],
                   "--param-file", controllers_yaml],
        output="screen",
        namespace=ns
    )

    cmd_vel_converter = Node(
        package='rc_car_test_description',
        executable='twist_to_joint_cmd',
        name='twist_to_joint_cmd',
        output='screen',
        parameters=[{
            'drive_sign': 1.0,     # 휠 출력 부호 (필요 시 -1)
        }],
        namespace=ns
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ns',
            default_value='rc_car',
            description='Namespace for RC car nodes'
        ),
        scan_rewriter,
        odom_rewriter,
        js_broadcaster,
        steering_spawner,
        wheel_spawner,
        cmd_vel_converter,
        

        # ros_gz_bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': ros_gz_bridge_yaml}],
            output='screen',
            namespace=ns
        ),
    ])
