from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os
from os.path import join

def generate_launch_description():
    # ===================== 1. 패키지 및 경로 설정 =====================
    share_dir = get_package_share_directory('jetank_description')
    rc_share_dir = get_package_share_directory('rc_car_test_description')
    rc_nav2_share_dir = get_package_share_directory('rc_nav2_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    models_path = os.path.join(share_dir, 'models')
    
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path
    # 모델 파일 경로
    jenga_sdf_path = os.path.join(share_dir, 'models', 'jenga_magnetic', 'model.sdf')
    world_path = os.path.join(share_dir, "worlds", "empty_world.sdf")
    
    # [NEW] 컨베이어 벨트 SDF 경로 설정
    # 주의: CMakeLists.txt에서 models 폴더가 install 되도록 설정되어 있어야 합니다.
    conveyor_sdf_path = os.path.join(share_dir, 'models', 'conveyor_belt', 'model.sdf')

    # Config 파일 경로
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    ros_gz_bridge_config = os.path.join(share_dir, 'config', 'gz_bridge.yaml')
    rc_ros_gz_bridge_config = os.path.join(rc_share_dir, 'config', 'ros_gz_bridge_gazebo.yaml')
    nav2_map_yaml = os.path.join(rc_nav2_share_dir, 'maps', 'jetank_map.yaml')

    # ===================== URDF/Xacro 파싱 =====================
    # 1) Jetank
    jetank_xacro_file = os.path.join(share_dir, 'urdf', 'jetank_main.xacro')
    jetank1_urdf = xacro.process_file(
        jetank_xacro_file, 
        mappings={'namespace': 'jetank1'}
    ).toxml()

    jetank2_urdf = xacro.process_file(
        jetank_xacro_file, 
        mappings={'namespace': 'jetank2'}
    ).toxml()

    # 2) Base Object
    base_urdf_file = os.path.join(share_dir, 'urdf', 'base.urdf')
    with open(base_urdf_file, 'r') as infp:
        base_desc = infp.read()

    # 3) RC Car
    rc_xacro_file = os.path.join(rc_share_dir, 'urdf', 'rc_car_test.xacro')
    rc_urdf = xacro.process_file(rc_xacro_file).toxml()

    # 4) Jenga URDF (RViz 시각화용)
    jenga_urdf_path = os.path.join(share_dir, 'urdf', 'jenga.urdf')
    try:
        with open(jenga_urdf_path, 'r') as infp:
            jenga_desc = infp.read()
    except FileNotFoundError:
        jenga_desc = ""
        print(f"[Warn] jenga.urdf not found at {jenga_urdf_path}")

    # ===================== 2. Launch Arguments =====================
    gui_arg = DeclareLaunchArgument(name='gui', default_value='True')
    robot_namespace_arg = DeclareLaunchArgument(name='robot_namespace', default_value='jetank')
    
    robot_namespace = LaunchConfiguration('robot_namespace') 
    rc_namespace = 'rc_car'
    base_namespace = 'base_obj'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ===================== 3. Gazebo 실행 =====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 4 {world_path}"}.items()
    )

    # ===================== 4. 기본 로봇 노드들 (RSP & Static TF) =====================
    
    # [Jetank] RSP & TF
    jetank1_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace="jetank1",
        parameters=[{'robot_description': jetank1_urdf, 'use_sim_time': use_sim_time, 'frame_prefix': 'jetank1/'}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'
    )
    tf_world_to_jetank1_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0', '0', '0', '0', '0', 'empty_world', 'jetank1/odom', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )
    tf_jetank1_odom_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0', '0.07', '0', '0', '0', 'jetank1/odom', 'jetank1/base_link', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )

    jetank2_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace="jetank2",
        parameters=[{'robot_description': jetank2_urdf, 'use_sim_time': use_sim_time, 'frame_prefix': 'jetank2/'}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'
    )
    tf_world_to_jetank2_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0.8', '0', '0', '0', '0', 'empty_world', 'jetank2/odom', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )
    tf_jetank2_odom_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0', '0.07', '0', '0', '0', 'jetank2/odom', 'jetank2/base_link', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )

    # [RC Car] RSP & TF
    rc_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=rc_namespace,
        parameters=[{'robot_description': rc_urdf, 'use_sim_time': use_sim_time, 'frame_prefix': 'rc_car/'}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'
    )
    tf_rc_odom_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0', '0', '0', '0', '0', 'rc_car/odom_rf2o', 'rc_car/base_link_rot', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )
    # Gazebo 스폰 위치를 odom_rf2o의 초기 원점으로 연결
    # NOTE: Avoid multiple parents for rc_car/odom_rf2o and base_link_rot.
    # tf_world_to_rc_odom_rf2o and rc_car_pose_tf_bridge are disabled to prevent TF conflicts.
    # AMCL (map->odom) and robot_state_publisher (odom->base) should provide the chain.

    # [Base Obj] RSP & TF
    base_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=base_namespace,
        parameters=[{'robot_description': base_desc, 'use_sim_time': use_sim_time, 'frame_prefix': 'base_obj/'}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'
    )
    tf_world_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['-0.05', '-0.25', '0', '3.14', '0', '0', 'empty_world', 'base_obj/base_link', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )

    # [NEW] [Conveyor Belt] TF (RSP는 URDF가 없으므로 생략하고 Static TF로 위치만 잡음)
    # 컨베이어 위치: x=0.5, y=0.0 (원하는 위치로 수정하세요)
    tf_world_to_conveyor = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['-0.05', '0.4', '0.0', '0', '0', '0', 'empty_world', 'conveyor_belt', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )
    # Cartographer map 원점을 rc_car 스폰 위치(0, 1, 0)와 일치시킴
    tf_world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.0', '1.0', '0.0', '0', '0', '0', 'empty_world', 'map', '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # ===================== Jenga Loop =====================
    jenga_nodes = []
    
    base_x = -0.05
    start_y = -0.16
    base_z = 0.12

    for i in range(1, 5): 
        jenga_name = f"jenga{i}"
        current_y = start_y + (i-1) * (-0.02)
        
        # 1. Gazebo Spawn
        spawn_node = TimerAction(
            period=10.0 + (i * 0.2),
            actions=[Node(
                package='ros_gz_sim', executable='create',
                arguments=[
                    "-file", jenga_sdf_path, 
                    "-name", jenga_name, 
                    "-x", str(base_x), "-y", str(current_y), "-z", str(base_z)
                ],
                output='screen'
            )]
        )
        jenga_nodes.append(spawn_node)

        # 2. RSP
        if jenga_desc:
            rsp_node = Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                name=f'rsp_{jenga_name}', 
                namespace=jenga_name,
                parameters=[{
                    'robot_description': jenga_desc,
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{jenga_name}/' 
                }],
                remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
                output='screen'
            )
            jenga_nodes.append(rsp_node)


    # ===================== 5. Spawning (나머지 로봇) =====================
    spawn_jetank1 = TimerAction(period=4.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-topic", "/jetank1/robot_description", "-name", "jetank1", "-x", "0.0", "-y", "0.0", "-z", "0.32"],
        output='screen'
    )])
    spawn_jetank2 = TimerAction(period=4.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-topic", "/jetank2/robot_description", "-name", "jetank2", "-x", "0.0", "-y", "0.8", "-z", "0.32"],
        output='screen'
    )])

    spawn_rc_car = TimerAction(period=5.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-topic", "/rc_car/robot_description", "-name", "rc_car", "-x", "-0.05", "-y", "1.0", "-z", "0.32"],
        output='screen'
    )])

    spawn_base = TimerAction(period=3.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-topic", "/base_obj/robot_description", "-name", "base_obj", "-x", "-0.05", "-y", "-0.25", "-z", "0.32", "-Y", "3.14"],
        output='screen'
    )])

    # [NEW] Conveyor Spawn
    # 위치: x=0.5, y=0.0 (TF와 동일하게 맞춰야 함)
    spawn_conveyor = TimerAction(period=3.5, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-file", conveyor_sdf_path, "-name", "conveyor_belt", "-x", "-0.05", "-y", "0.4", "-z", "0.0"],
        output='screen'
    )])

    # ===================== 6. Bridges & Extras =====================
    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        namespace=robot_namespace,
        parameters=[{'use_sim_time': use_sim_time, 'config_file': ros_gz_bridge_config}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static'), ('/clock', '/clock')],
        output='screen'
    )

    rc_ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        namespace=rc_namespace,
        parameters=[{'use_sim_time': use_sim_time, 'config_file': rc_ros_gz_bridge_config}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        output='screen'
    )

    rc_rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        namespace=rc_namespace,
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'laser_scan_topic': 'scan',  # resolves to /rc_car/scan
            'odom_topic': 'odom_rf2o',   # resolves to /rc_car/odom_rf2o
            'publish_tf': True,
            'base_frame_id': 'rc_car/base_link_rot',
            'odom_frame_id': 'rc_car/odom_rf2o',
            'init_pose_from_topic': '',
            'freq': 30.0
        }]
    )
    # 모든 스폰/컨트롤러 준비 뒤 RF2O 시작
    rc_rf2o_delayed = TimerAction(period=25.0, actions=[rc_rf2o_node])

    rc_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(rc_share_dir, 'launch', 'rc_car_controllers.launch.py')),
        launch_arguments={
            'ns': rc_namespace
        }.items()
    )
    rc_controllers_delayed = TimerAction(period=8.0, actions=[rc_controllers_launch])


    jetank1_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(share_dir, 'launch', 'jetank_controllers.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ns': 'jetank1'  # [수정됨] 받는 쪽 인자 이름이 'ns'인 경우
            # 만약 받는 쪽 인자 이름이 'robot_namespace'라면 아래 주석 해제
            # 'robot_namespace': 'jetank2' 
        }.items()
    )

    jetank1_delayed_controllers = TimerAction(period=15.0, actions=[jetank1_controllers_launch])

    jetank2_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(share_dir, 'launch', 'jetank_controllers.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ns': 'jetank2'  # [수정됨] 받는 쪽 인자 이름이 'ns'인 경우
            # 만약 받는 쪽 인자 이름이 'robot_namespace'라면 아래 주석 해제
            # 'robot_namespace': 'jetank2' 
        }.items()
    )

    jetank2_delayed_controllers = TimerAction(period=25.0, actions=[jetank2_controllers_launch])


    rviz_node = Node(
        package='rviz2', executable='rviz2', namespace=robot_namespace,
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        output='screen'
    )

    rqt_node = Node(
        package="rqt_gui", executable="rqt_gui", output="screen", namespace="jetank",
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(rc_nav2_share_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'False',
            'map': nav2_map_yaml,
            'use_namespace': 'False',
            'use_rviz': 'True',
            'cmd_vel_bridge_target': '/rc_car/cmd_vel',
        }.items()
    )
    nav2_delayed = TimerAction(period=28.0, actions=[nav2_launch])

    # ===============python으로 jenga detach하기
    script_path = os.path.join(
            os.path.expanduser('~'), 
            'jetank_ws/src/jetank_description/scripts/detach.py'
        )

    # 2. 실행 명령 및 15초 타이머 설정
    delayed_detach_action = TimerAction(
        period=20.0,  # 15초 대기
        actions=[
            ExecuteProcess(
                cmd=['python3', script_path],
                output='screen'
            )
        ]
    )

    # ===================== 최종 리스트 병합 =====================
    nodes_to_run = [
        gui_arg, robot_namespace_arg,
        gazebo,
        jetank1_rsp,jetank2_rsp, rc_rsp, base_rsp,
        tf_world_to_jetank1_odom, tf_jetank1_odom_to_base,
        tf_world_to_jetank2_odom, tf_jetank2_odom_to_base,
        tf_rc_odom_to_base, rc_rf2o_delayed,
        tf_world_to_base,
        tf_world_to_conveyor,  # [NEW] TF 추가
        tf_world_to_map,
        spawn_jetank1, spawn_jetank2, spawn_rc_car, spawn_base, 
        spawn_conveyor,        # [NEW] Spawn 추가
        ros_gz_bridge, rc_ros_gz_bridge, rc_controllers_delayed, 
        jetank1_delayed_controllers, jetank2_delayed_controllers,
        rviz_node, rqt_node,
        nav2_delayed,
        delayed_detach_action
    ]
    
    nodes_to_run.extend(jenga_nodes)

    return LaunchDescription(nodes_to_run)
