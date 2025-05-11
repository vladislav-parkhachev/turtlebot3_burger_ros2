import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('joint_state_publisher', default_value='true',
                          choices=['true', 'false'],
                          description='Start joint_state_publisher.'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('rviz_config', default_value=os.path.join(
        get_package_share_directory('robot_description'), 'rviz', 'robot_view.rviz'),
                          description='Rviz config.'),
]

def generate_launch_description():
    pkg_turtlebot3_description = get_package_share_directory('robot_description')
    xacro_file =  os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3.urdf.xacro')
    conf_file =  os.path.join(pkg_turtlebot3_description, 'config', 'robot_components.yaml')
    config = yaml.safe_load(Path(conf_file).read_text())
    doc = xacro.process_file(xacro_file, mappings=config)
    robot_description = doc.toprettyxml(indent='  ')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}, 
            {'robot_description': robot_description}
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time' : LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('joint_state_publisher'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)

    return ld