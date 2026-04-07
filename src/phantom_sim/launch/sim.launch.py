import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_pkg = get_package_share_directory("phantom_model")
    sim_pkg = get_package_share_directory("phantom_sim")
    config = os.path.join(model_pkg, "config", "phantom_params.yaml")
    rviz_cfg = os.path.join(sim_pkg, "rviz", "phantom.rviz")

    return LaunchDescription([
        Node(
            package="phantom_controllers",
            executable="base_controller_node",
            name="gravity_compensator",
            parameters=[{
                "config_file": config,
                "gravity_compensation": True,
            }],
            output="screen",
        ),
        Node(
            package="phantom_sim",
            executable="simulator_node",
            name="phantom_simulator",
            parameters=[{
                "config_file": config,
                "integration_dt": 0.0002,
                "publish_rate": 1000.0,
                "wait_for_input": True,
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
        ),
    ])
