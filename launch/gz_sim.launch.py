import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ----- Paths ------
    this_share = get_package_share_directory("gz_px4_env") # Package install/share directory 
    default_bridge_config = os.path.join(this_share, "config", "bridge_config.yaml") # Path to gz_ros_bridge config file
    default_px4_dir = os.path.expanduser("~/PX4-Autopilot") # PX4 root directory 

    # ----- Launch Configs ------
    bridge_config = LaunchConfiguration("bridge_config")
    px4_dir = LaunchConfiguration("px4_dir")

    px4_build_dir = PathJoinSubstitution([px4_dir, "build", "px4_sitl_default"])
    px4_exe = PathJoinSubstitution([px4_build_dir, "bin", "px4"])

    # ----- Launch Args ------
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

    # ----- ROS2 <-> Gazebo bridge (optional) ------
    bridge_node = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        output = "screen",
        parameters = [{"config_file": bridge_config}],
    )

    # ----- PX4 env: let PX4 start Gazebo + x500, but keep HOME/DISPLAY ------
    set_px4_autostart = SetEnvironmentVariable(
        name = "PX4_SYS_AUTOSTART",
        value = "4001",  # x500 airframe
    )
    set_px4_simulator = SetEnvironmentVariable(
        name = "PX4_SIMULATOR",
        value = "gz",    # Gazebo (GZ) simulator
    )
    # New-style env var used by current PX4 for GZ Sim
    set_px4_sim_model = SetEnvironmentVariable(
        name = "PX4_SIM_MODEL",
        value = "x500",
    )

    # Extend PATH so rcS can find px4-alias.sh etc, but keep existing PATH
    set_px4_path = SetEnvironmentVariable(
        name = "PATH",
        value = [
            EnvironmentVariable("PATH"),
            TextSubstitution(text = ":"),
            PathJoinSubstitution([px4_build_dir, "bin"]),
        ],
    )

    # ----- PX4 SITL ------
    px4_sitl = ExecuteProcess(
        cmd = [px4_exe],
        cwd = px4_build_dir,
        output = "screen",
    )

    # ----- Micro XRCE-DDS Agent ------
    microagent = ExecuteProcess(
        cmd = ["MicroXRCEAgent", "udp4", "-p", "8888"],
        output = "screen",
    )

    return LaunchDescription(
        [
            decl_bridge_config_arg,
            decl_px4_dir_arg,
            set_px4_autostart,
            set_px4_simulator,
            set_px4_sim_model,
            set_px4_path,
            bridge_node,
            px4_sitl,
            microagent,
        ]
    )
