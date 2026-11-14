import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("crane_rrt")
    config_file = os.path.join(pkg_share, "config", "rrt_exploration_config.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "rrt_exploration.rviz")

    arena_arg = DeclareLaunchArgument(
        "arena",
        default_value="Arena1_ImageIndustrial",
        description=(
            "Obstacle arena name or YAML path: Arena1_ImageIndustrial, "
            "Arena2_SlalomCorridor, Arena3_DensePillarYard, "
            "Arena4_PayloadClutterCorridor, or none"
        ),
    )
    with_rviz_arg = DeclareLaunchArgument(
        "with_rviz",
        default_value="true",
        description="Launch RViz2 visualization",
    )
    with_gui_arg = DeclareLaunchArgument(
        "with_gui",
        default_value="true",
        description="Launch interactive PyQt5 RRT control GUI",
    )

    return LaunchDescription([
        arena_arg,
        with_rviz_arg,
        with_gui_arg,
        Node(
            package="crane_rrt",
            executable="rrt_exploration_node",
            name="rrt_exploration_node",
            output="screen",
            parameters=[config_file, {"arena_file": LaunchConfiguration("arena")}],
        ),
        Node(
            package="crane_rrt",
            executable="visualizer_node",
            name="visualizer_node",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="crane_rrt",
            executable="arena_visualizer_node",
            name="arena_visualizer_node",
            output="screen",
            parameters=[config_file, {"arena_file": LaunchConfiguration("arena")}],
        ),
        Node(
            package="crane_rrt",
            executable="rrt_control_gui.py",
            name="rrt_control_gui_node",
            output="screen",
            parameters=[{"arena_file": LaunchConfiguration("arena")}],
            condition=IfCondition(LaunchConfiguration("with_gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("with_rviz")),
        ),
    ])

