import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("phantom_sim")
    config = os.path.join(pkg, "config", "phantom_params.yaml")
    rviz_cfg = os.path.join(pkg, "rviz", "phantom.rviz")

    return LaunchDescription([
        Node(
            package="phantom_sim",
            executable="gravity_comp_node",
            name="gravity_compensator",
            parameters=[{"config_file": config}],
            output="screen",
        ),
        Node(
            package="phantom_sim",
            executable="simulator_node",
            name="phantom_simulator",
            parameters=[{"config_file": config}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
        ),
    ])
