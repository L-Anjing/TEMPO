"""Single MID-360 driver, with parameter and network files owned by this package."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def config(filename):
        return PathJoinSubstitution([FindPackageShare("livox_ros_driver2"), "config", filename])

    return LaunchDescription([
        DeclareLaunchArgument("driver_params_file", default_value=config("MID360_params.yaml")),
        DeclareLaunchArgument("lidar_config_file", default_value=config("MID360_config.json")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node",
            name="livox_driver_node", output="screen",
            parameters=[
                ParameterFile(LaunchConfiguration("driver_params_file"), allow_substs=True),
                {"user_config_path": LaunchConfiguration("lidar_config_file"),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            ],
        ),
    ])
