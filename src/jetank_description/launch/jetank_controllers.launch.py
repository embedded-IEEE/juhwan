import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , PythonExpression

def generate_launch_description():


    jetank_package = get_package_share_directory('jetank_description')
    jetank_controllers_config = os.path.join(jetank_package,"config","jetank_controllers.yaml")

    robot_namespace = LaunchConfiguration('ns')
    robot_namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='[ARG] required namespace to keep nodes and topics separate when running multiple robots in the same simulation',
        default_value='jetank'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='[ARG] tells the robot_state_publisher to use the simulation time or just unix timestamp'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'joint_state_broadcaster',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
            ]
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'diff_drive_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
            ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'arm_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
            ]
     )
    
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=[
            'gripper_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10',
        ]
    )

    joint_state_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                #diff_drive_controller_spawner,
                arm_controller_spawner,
                #gripper_controller_spawner
            ]
    ))     
   
    return LaunchDescription([

        robot_namespace_arg,
        use_sim_time_arg,


        joint_state_broadcaster_spawner,
        joint_state_event_handler,
    ])
