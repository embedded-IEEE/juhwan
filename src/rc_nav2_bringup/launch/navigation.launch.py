import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 패키지 공유 디렉토리 경로 자동 획득
    bringup_dir = get_package_share_directory('nav2_bringup')
    rc_nav2_bringup_dir = get_package_share_directory('rc_nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # BT XML 파일 경로 설정 (상대 경로 문제 해결)
    default_bt_xml_path = os.path.join(rc_nav2_bringup_dir, 'config', 'simple_nav_to_pose.xml')
    default_nav_through_poses_bt_xml_path = os.path.join(rc_nav2_bringup_dir, 'config', 'simple_nav_through_poses.xml')

    # Launch Configuration 변수 설정
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    cmd_vel_bridge_target = LaunchConfiguration('cmd_vel_bridge_target')

    # Launch Arguments 선언
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for Nav2 nodes')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='False', description='Whether to apply a namespace')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM')
    
    # 맵 경로: 패키지 내부의 maps 폴더를 기본값으로 설정 (필요시 수정)
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(rc_nav2_bringup_dir, 'maps', 'jetank_map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock if true')

    # 파라미터 파일 경로 자동 설정
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(rc_nav2_bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True', description='Automatically startup the nav2 stack')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Launch RViz2')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file')

    declare_cmd_vel_bridge_target_cmd = DeclareLaunchArgument(
        'cmd_vel_bridge_target',
        default_value='/rc_car/cmd_vel',
        description='Target topic to republish /cmd_vel to')

    # BT XML 경로를 nav2_params.yaml에 주입해서 절대경로 사용하도록 변환
    bt_param_overrides = {
        'default_bt_xml_filename': default_bt_xml_path,
        'default_nav_to_pose_bt_xml': default_bt_xml_path,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml_path,
    }
    rewritten_params_file = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=bt_param_overrides,
        convert_types=True)

    # Nav2 Bringup 실행 (수정된 파라미터 파일을 인자로 주입)
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': rewritten_params_file,
            'autostart': autostart,
            'use_rviz': use_rviz,
            'use_composition': use_composition,
            'namespace': namespace,
            'use_namespace': use_namespace,
            # 아래 파라미터들이 YAML 내부의 절대 경로 설정을 덮어씌웁니다.
            'default_bt_xml_filename': default_bt_xml_path,
            'default_nav_to_pose_bt_xml': default_bt_xml_path,
            'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml_path
        }.items())

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    cmd_vel_bridge = Node(
        package='rc_nav2_bringup',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'target_topic': cmd_vel_bridge_target}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
    )

    # LaunchDescription 구성
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_cmd_vel_bridge_target_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(cmd_vel_bridge)

    return ld
