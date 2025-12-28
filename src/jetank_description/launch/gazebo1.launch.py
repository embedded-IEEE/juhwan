from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 모델 파일 경로
    jenga_sdf_path = os.path.join(share_dir, 'models', 'jenga_magnetic', 'model.sdf')
    world_path = os.path.join(share_dir, "worlds", "empty_world.sdf")
    
    # Config 파일 경로
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    ros_gz_bridge_config = os.path.join(share_dir, 'config', 'gz_bridge.yaml')
    rc_ros_gz_bridge_config = os.path.join(rc_share_dir, 'config', 'ros_gz_bridge_gazebo.yaml')

    # ===================== URDF/Xacro 파싱 =====================
    # 1) Jetank
    jetank_xacro_file = os.path.join(share_dir, 'urdf', 'jetank_main.xacro')
    jetank_urdf = xacro.process_file(jetank_xacro_file).toxml()

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
    jetank_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=robot_namespace,
        parameters=[{'robot_description': jetank_urdf, 'use_sim_time': use_sim_time, 'frame_prefix': 'jetank/'}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'
    )
    tf_world_to_jetank_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0', '0', '0', '0', '0', 'empty_world', 'jetank/odom', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )
    tf_jetank_odom_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0', '0.07', '0', '0', '0', 'jetank/odom', 'jetank/base_link', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )

    # [RC Car] RSP & TF
    rc_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=rc_namespace,
        parameters=[{'robot_description': rc_urdf, 'use_sim_time': use_sim_time, 'frame_prefix': 'rc_car/'}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'
    )
    tf_world_to_rc_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0.0', '0.2', '0', '0', '0', '0', 'empty_world', 'rc_car/odom', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )
    tf_rc_odom_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments=['0', '0', '0', '0', '0', '0', 'rc_car/odom', 'rc_car/base_link', '--ros-args', '-p', 'use_sim_time:=true'], 
        output='screen'
    )

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


    # ===================== [수정됨] Jenga Loop =====================
    # 젠가 1~10을 world 좌표계에 직접 연결하여 TF 트리를 완성합니다.
    # ===================== [수정됨] Jenga Loop (Static TF 제거) =====================
    # ===================== [수정됨] Jenga Loop (Static TF 제거) =====================
    jenga_nodes = []
    
    base_x = -0.05
    start_y = -0.16
    base_z = 0.12

    for i in range(1, 11): 
        jenga_name = f"jenga{i}"
        current_y = start_y + (i-1) * (-0.02)
        
        # 1. Gazebo Spawn (모델 생성) - 이건 유지!
        spawn_node = TimerAction(
            period=10.0 + (i * 0.5),
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

        # [삭제됨] static_transform_publisher 
        # -> 이제 gz_bridge가 실시간 위치를 /tf로 쏴주므로, 
        #    여기서 고정 위치를 뿌리면 충돌납니다. 과감히 삭제!

        # 2. RSP (RViz 시각화용) - 유지
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
    spawn_jetank = TimerAction(period=4.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-topic", "/jetank/robot_description", "-name", "jetank", "-x", "0.0", "-y", "0.0", "-z", "0.32"],
        output='screen'
    )])

    spawn_rc_car = TimerAction(period=5.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-topic", "/rc_car/robot_description", "-name", "rc_car", "-x", "0.0", "-y", "0.2", "-z", "0.32"],
        output='screen'
    )])

    spawn_base = TimerAction(period=3.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=["-topic", "/base_obj/robot_description", "-name", "base_obj", "-x", "-0.05", "-y", "-0.25", "-z", "0.32", "-Y", "3.14"],
        output='screen'
    )])


    # ===================== 6. Bridges & Extras =====================
    # use_sim_time이 True로 잘 설정되어 있습니다.
    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        namespace=robot_namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '-p', PythonExpression(["'", 'config_file:=', ros_gz_bridge_config, "'"])],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static'), ('/clock', '/clock')],
        output='screen'
    )

    rc_ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        namespace=rc_namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '-p', PythonExpression(["'", 'config_file:=', rc_ros_gz_bridge_config, "'"])],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        output='screen'
    )

    ros_gz_image_bridge = Node(
        package='ros_gz_image', executable='image_bridge', namespace=robot_namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[PythonExpression(["'/", robot_namespace, '/camera/image_raw', "'"]), "camera/image_raw"],
        output='screen'
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(share_dir, 'launch', 'jetank_controllers.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'robot_namespace': robot_namespace}.items()
    )
    delayed_controllers = TimerAction(period=8.0, actions=[controllers_launch])

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

    # ===================== 최종 리스트 병합 =====================
    nodes_to_run = [
        gui_arg, robot_namespace_arg,
        gazebo,
        jetank_rsp, rc_rsp, base_rsp,
        tf_world_to_jetank_odom, tf_jetank_odom_to_base,
        tf_world_to_rc_odom, tf_rc_odom_to_base,
        tf_world_to_base,
        spawn_jetank, spawn_rc_car, spawn_base,
        ros_gz_bridge, rc_ros_gz_bridge, ros_gz_image_bridge,
        delayed_controllers, rviz_node, rqt_node
    ]
    
    # 젠가 노드들(10개 분량)을 리스트에 추가
    nodes_to_run.extend(jenga_nodes)

    return LaunchDescription(nodes_to_run)