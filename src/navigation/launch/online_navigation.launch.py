"""Navigation only: consumes external odometry/clouds and outputs paths for MPC."""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _nodes(context):
    def enabled(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        return value == "true"

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    components = []
    with open(LaunchConfiguration("pointlio_params_file").perform(context), encoding="utf-8") as stream:
        lio = yaml.safe_load(stream)["/**"]["ros__parameters"]
    calibration = lio["robot_extrinsic"]
    frames = lio["frames"]
    with open(LaunchConfiguration("params_file").perform(context), encoding="utf-8") as stream:
        nav = yaml.safe_load(stream)["planner_server"]["ros__parameters"]["MincoPlanner"]
    if frames["odom"] != nav["frames"]["rog_frame"] or frames["odom"] != nav["frames"]["map_frame"]:
        raise ValueError("Point-LIO and navigation must use the same world frame")
    sensor_params = {
        "MincoPlanner.frames.robot_frame": frames["robot"],
        "MincoPlanner.rog_map.ros_callback.robot_frame": frames["robot"],
        "MincoPlanner.rog_map.sensor_in_robot": calibration["lidar_to_robot_T"],
        "MincoPlanner.rog_map.sensor_in_robot_R": calibration["lidar_to_robot_R"],
        "MincoPlanner.rog_map.cloud_filter.position_frame": frames["robot"],
    }
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={"use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_base_frame": frames["robot"]},
        convert_types=True), allow_substs=False)
    components.append(ComposableNode(
        package="nav2_planner", plugin="nav2_planner::PlannerServer", name="planner_server",
        parameters=[params, sensor_params], extra_arguments=[{"use_intra_process_comms": False}],
    ))
    actions = [
        ComposableNodeContainer(
            name="online_navigation_container", namespace="",
            package="navigation", executable="online_navigation_container",
            prefix=LaunchConfiguration("container_prefix"),
            arguments=["--ros-args", "--log-level", "warn",
                       "--log-level", "navigation.path:=info"],
            # Costmap2DROS is constructed inside PlannerServer and uses process ROS arguments.
            parameters=[params],
            # Let map worker threads sleep between parallel regions.
            additional_env={"OMP_WAIT_POLICY": "PASSIVE"},
            composable_node_descriptions=components, output="screen",
        ),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_planning", output="screen",
             arguments=["--ros-args", "--log-level", "warn"],
             parameters=[{"use_sim_time": sim_time, "autostart": True,
                          "node_names": ["planner_server"]}]),
    ]
    if enabled("rviz"):
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz_online_navigation",
            arguments=["-d", LaunchConfiguration("rviz_config_file"),
                       "--ros-args", "--log-level", "warn"],
            parameters=[{"use_sim_time": sim_time}], output="screen",
        ))
    return actions


def generate_launch_description():
    def resource(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("container_prefix", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=resource("navigation", "params", "online_planner.yaml")),
        DeclareLaunchArgument("pointlio_params_file", default_value=resource("point_lio", "config", "mapping", "mid360.yaml")),
        DeclareLaunchArgument("rviz_config_file", default_value=resource("navigation", "config", "online_navigation.rviz")),
        OpaqueFunction(function=_nodes),
    ])
