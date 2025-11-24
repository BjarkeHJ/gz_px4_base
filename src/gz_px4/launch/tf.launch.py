
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("gz_px4")
    config_file = os.path.join(pkg_share, "config", "gz_config.yaml")

    with open(config_file, 'r') as f:
        cfg = yaml.safe_load(f)
        
    urdf_path = os.path.join(pkg_share, cfg["transform_path"])

    odom_tf = Node(
        package="gz_px4",
        executable="odom_tf_pub",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    static_tf = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="static_tf",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": open(urdf_path).read()},
        ],
    )

    return LaunchDescription([odom_tf, static_tf])