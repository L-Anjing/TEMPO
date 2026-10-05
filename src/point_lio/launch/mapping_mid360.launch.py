from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    params_file = LaunchConfiguration("params_file")

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    point_lio_dir = get_package_share_directory("point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mapping", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        respawn=True,
        respawn_delay=5.0,
        namespace=namespace,
        parameters=[params_file, {"use_sim_time": sim_time}],
        remappings=remappings,
        output="screen",
        # OpenMP's default ACTIVE spin-wait competes with the tracking thread when
        # mapping.batch.omp=true; force PASSIVE unless the caller already chose one.
        additional_env={"OMP_WAIT_POLICY": "PASSIVE"},
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        parameters=[{"use_sim_time": sim_time}],
        remappings=remappings,
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config_file"),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_params_file)
    ld.add_action(DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution([point_lio_dir, "rviz_cfg", "mapping_mid360.rviz"]),
        description="Path to the Point-LIO mapping RViz configuration",
    ))
    ld.add_action(start_point_lio_node)
    ld.add_action(start_rviz_node)

    return ld
