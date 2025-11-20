import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ----- Paths ------
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    this_share = get_package_share_directory("learn_gz")

    default_world = os.path.join(this_share, "worlds", "minimal_empty.sdf")
    default_bridge_config = os.path.join(this_share, "config", "bridge_config.yaml")
    default_px4_dir = os.path.expanduser("~/PX4-Autopilot")

    # ----- Launch Configs ------
    world = LaunchConfiguration("world")
    bridge_config = LaunchConfiguration("bridge_config")
    px4_dir = LaunchConfiguration("px4_dir")

    px4_build_dir = PathJoinSubstitution([px4_dir, "build", "px4_sitl_default"])
    px4_models_path = PathJoinSubstitution([px4_dir, "Tools", "simulation", "gz", "models"])

    # ----- Launch Args ------
    decl_world_arg = DeclareLaunchArgument(
        name = "world",
        default_value = default_world,
        description = "Path to the Gazebo world file",
    )

    decl_bridge_config_arg = DeclareLaunchArgument(
        name = "bridge_config",
        default_value = default_bridge_config,
        description = "Path to ros_gz_bridge config file",
    )

    decl_px4_dir_arg = DeclareLaunchArgument(
        name = "px4_dir",
        default_value = default_px4_dir,
        description = "Path to PX4-Autopilot root directory",
    )

    # ----- Environment Variables so GZ can find PX4 models
    set_ign_resource = SetEnvironmentVariable(
        name = "IGN_GAZEBO_RESOURCE_PATH",
        value = px4_models_path,
    )
    set_gz_resource = SetEnvironmentVariable(
        name = "GZ_SIM_RESOURCE_PATH",
        value = px4_models_path,
    )

    # ----- Include gz_sim launch file from ros_gz_sim ------
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments = {"gz_args": world}.items(),
    )

    # ----- Spawn x500 Quadcopter
    x500_model_path = PathJoinSubstitution(
        [px4_models_path, "x500", "model.sdf"]
    )

    quadcopter_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "x500",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "1.0",
            "-file", x500_model_path,
        ],
    )

    # ----- ROS2 <-> Gazebo bridge ------
    bridge_node = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        output = "screen",
        parameters = [{"config_file": bridge_config}]
    )

    # ----- PX4 SITL ------
    px4_exe = PathJoinSubstitution([px4_build_dir, "bin", "px4"])

    px4_sitl = ExecuteProcess(
        cmd = [px4_exe],
        cwd = px4_build_dir,
        env = {
            # x500 airframe
            "PX4_SYS_AUTOSTART": "4001",
            # Use GZ (Gazebo) + x500 model
            "PX4_SIMULATOR": "gz",
            "PX4_GZ_MODEL": "x500",
            "PX4_GZ_STANDALONE": "1",

            # IMPORTANT: allow rcS to find px4-alias.sh
            "PATH": [
                EnvironmentVariable("PATH"),
                TextSubstitution(text=":"),
                PathJoinSubstitution([px4_build_dir, "bin"]),
            ],
        },
        output="screen",
    )

    launch_description = [
            decl_world_arg,
            decl_bridge_config_arg,
            decl_px4_dir_arg,
            set_ign_resource,
            set_gz_resource,
            gz_sim_launch,
            quadcopter_spawn,
            bridge_node,
            px4_sitl,
        ]
    
    return LaunchDescription(launch_description)
