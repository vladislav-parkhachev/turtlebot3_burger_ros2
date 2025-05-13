import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from launch.event_handlers import OnProcessExit

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('rviz_config', default_value=os.path.join(
        get_package_share_directory('robot_simulation'), 'rviz', 'simulation.rviz'),
        description='Rviz config.'),
    DeclareLaunchArgument('world', default_value=os.path.join(
        get_package_share_directory('robot_simulation'), 'worlds','empty_world.world'),
        description='World to load')
]

def generate_launch_description():
    
    robot_description_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                    'launch',
                    'turtlebot3_description.launch.py'
            ])
        ]),
        launch_arguments = [
            ('rviz', LaunchConfiguration('rviz')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('rviz_config', LaunchConfiguration('rviz_config')),
            ('joint_state_publisher', 'false'),
            ('simulation', 'true')
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('robot_simulation'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )
    
    world = LaunchConfiguration('world')

    gazeboLaunch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            '--controller-ros-args',
            '-r /diff_drive_controller/cmd_vel:=/cmd_vel',
        ],
    )

    bridge_params = os.path.join(get_package_share_directory('robot_simulation'), 'parameters', 'bridge_parameters.yaml')

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', 
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # package_name_gazebo = 'mycobot_gazebo'
    # gazebo_models_path = 'models'
    # pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    # gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)

    # set_env_vars_resources = AppendEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     gazebo_models_path)
    
    delayed_start = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_base_controller_spawner]))

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description_launch_file)
    # ld.add_action(set_env_vars_resources)
    ld.add_action(gazeboLaunch)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(gz_spawn_entity)
    ld.add_action(delayed_start)
    ld.add_action(load_joint_state_broadcaster_cmd)

    return ld