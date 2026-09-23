from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = (
        get_package_share_directory("stl_to_grid_map") + "/config/rmuc2025.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_file"),
            DeclareLaunchArgument("output_prefix", default_value="RMUC2025"),
            Node(
                package="stl_to_grid_map",
                executable="stl_to_grid_node",
                name="stl_to_grid_map",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "input_file": LaunchConfiguration("input_file"),
                        "output_prefix": LaunchConfiguration("output_prefix"),
                    },
                ],
            )
        ]
    )
