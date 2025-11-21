import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gz_px4_env')
    
    config_file = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")
    
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros_bridge",
        output="screen",
        arguments=[
            '--ros-args',
            '-p', f'config_file:={config_file}',
        ],
    )

    return LaunchDescription([bridge_node])


