from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os
from os.path import join

def generate_launch_description():
    share_dir = get_package_share_directory('jetank_description')

    jenga_model_path = os.path.join(
        share_dir, 'models', 'jenga_magnetic', 'model.sdf'
    )

    # ========== 1. 기존 Jetank URDF 로딩 (Xacro) ==========
    xacro_file = os.path.join(share_dir, 'urdf', 'jetank_main.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # ========== [추가됨] 2. Base URDF 로딩 ==========
    # base.urdf 파일이 share_dir/urdf/ 폴더 안에 있다고 가정합니다.
    base_urdf_file = os.path.join(share_dir, 'urdf', 'base.urdf')
    
    # 일반 URDF 파일인 경우 파일 내용을 그대로 읽습니다.
    with open(base_urdf_file, 'r') as infp:
        base_desc = infp.read()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    ros_gz_bridge_config = os.path.join(share_dir, 'config', 'gz_bridge.yaml')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ========== Launch Arguments ==========
    gui_arg = DeclareLaunchArgument(name='gui', default_value='True')
    
    # Jetank 네임스페이스
    robot_namespace_arg = DeclareLaunchArgument(
        name='robot_namespace',
        description='로봇 네임스페이스',
        default_value='jetank'
    )
    
    # [추가됨] Base 네임스페이스 (충돌 방지용)
    base_namespace = 'base_obj'

    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ========== Nodes ==========
    
    # 1. Gazebo 실행
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
    #     launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
    # )

    world_path = os.path.join(share_dir, "worlds", "empty_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_path}"
        }.items()
    )

    spawn_robot = TimerAction( period=4.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", PythonExpression(["'",robot_namespace,'/robot_description',"'"]),
                "-name", robot_namespace,
                "-allow_renaming", "false",
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    spawn_jenga = TimerAction(period=6.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-file", jenga_model_path,   # SDF 파일 경로
                "-name", "jenga",
                "-allow_renaming", "false",
                "-x", "-0.05",   # 탁자 위에 올라가도록 위치 조정 (예시 값)
                "-y", "-0.2",
                "-z", "0.32",
                "-Y", "0.0",
            ],
            output='screen'
        )
    ])

    # [추가됨] 2-2. Base Spawn
    # Jetank와 겹치지 않도록 위치(x, y, z)를 조정해주세요.
    spawn_base = TimerAction(period=3.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", f"/{base_namespace}/robot_description", # Base의 RSP 토픽을 구독
                "-name", base_namespace,
                "-x", "-0.05",  # 예: Jetank 옆 1m 지점
                "-y", "-0.3",
                "-z", "0.32",  # 바닥
                "-Y", "3.14"
            ],
            output='screen'
        )]
    )

    # 3-1. Jetank RSP
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time} 
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    # [추가됨] 3-2. Base RSP
    # base.urdf 내용을 발행하여 Gazebo가 spawn 할 때 참조하고, RViz에서도 볼 수 있게 합니다.
    base_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=base_namespace, # 별도의 네임스페이스 사용
        parameters=[
            {'robot_description': base_desc},
            {'use_sim_time': use_sim_time} 
        ],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], # TF는 전역으로 합칠지 선택 (보통 합침)
        output='screen'
    )

    # 4. ROS GZ Bridge (Jetank용)
    # 4. ROS GZ Bridge (수정됨)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_namespace,
        # [중요] Bridge는 시뮬레이션 시간을 '만드는' 녀석이므로 False여야 합니다!
        # True로 두면 자기 꼬리를 물고 돕니다.
        parameters=[{'use_sim_time': False}], 
        arguments=[
            '--ros-args',
            '-p', PythonExpression(["'",'config_file:=',ros_gz_bridge_config,"'"]),
        ],
        remappings=[('/tf','/tf'), ('/tf_static','/tf_static'), ('/clock','/clock')],
        output='screen'
    )

    ros_gz_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace=robot_namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[PythonExpression(["'/",robot_namespace,'/camera/image_raw',"'"]), "camera/image_raw"],
        output='screen'
    )

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_namespace,
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/robot_description', 'robot_description')
        ]
    )

    # 컨트롤러 실행
    controllers_launch_file = join(share_dir, 'launch', 'jetank_controllers.launch.py')
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controllers_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_namespace': robot_namespace
        }.items()
    )
    delayed_controllers_launch = TimerAction(period=8.0, actions=[controllers_launch])


    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        output="screen",
        namespace="jetank",
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # ========== LaunchDescription ==========
    return LaunchDescription([
        gui_arg,
        robot_namespace_arg,
        
        # Gazebo
        gazebo,
        spawn_robot, # Jetank 소환
        spawn_base,  # [추가됨] Base 소환
        spawn_jenga,   # [추가] 젠가 스폰

        
        ros_gz_bridge,
        ros_gz_image_bridge_node,

        # ROS Nodes
        robot_state_publisher_node, # Jetank RSP
        base_state_publisher_node,  # [추가됨] Base RSP
        delayed_controllers_launch,
        rviz_node,
        rqt

    ])
