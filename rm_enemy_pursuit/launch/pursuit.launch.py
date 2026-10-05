from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rm_enemy_pursuit'))
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('simulate', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('bearing_deg', default_value='180.0'),
        Node(package='rm_enemy_pursuit', executable='pursuit_manager', output='screen',
             parameters=[str(share/'config/pursuit.yaml'), clock]),
        Node(package='rm_enemy_pursuit', executable='enemy_simulator', output='screen',
             condition=IfCondition(LaunchConfiguration('simulate')),
             parameters=[clock, {'bearing_deg': ParameterValue(LaunchConfiguration('bearing_deg'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share/'config/pursuit.rviz')], parameters=[clock])])
