"""包含原工程启动入口；目标输入由本包 RViz 的 2D Goal Pose 接管。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    own = Path(get_package_share_directory('rm_enemy_pursuit'))
    bringup = Path(get_package_share_directory('rm_nav_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='RMUL', choices=['RMUL', 'RMUC']),
        DeclareLaunchArgument('mode', default_value='nav', choices=['nav', 'mapping']),
        DeclareLaunchArgument('profile', default_value='baseline',
                              choices=['baseline', 'buffer5', 'window08', 'resolution0025', 'combined']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('simulate', default_value='true'),
        DeclareLaunchArgument('bearing_deg', default_value='180.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/bringup_sim.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'mode': LaunchConfiguration('mode'),
                              'lio': 'fastlio', 'localization': 'slam_toolbox',
                              'gui': LaunchConfiguration('gui'), 'nav_rviz': 'false', 'lio_rviz': 'false',
                              'localization_params_file': PathJoinSubstitution([
                                  str(bringup/'config/simulation/localization_trials'),
                                  [LaunchConfiguration('profile'), '.yaml']])}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(own/'launch/pursuit.launch.py')),
            launch_arguments={'use_sim_time':'true', 'rviz':LaunchConfiguration('rviz'),
                              'simulate':LaunchConfiguration('simulate'),
                              'bearing_deg':LaunchConfiguration('bearing_deg')}.items())])
