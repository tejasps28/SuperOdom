import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    config_path = get_share_file(
        package_name="super_odometry",
        file_name="config/tartan_air.yaml",
    )
    calib_path = get_share_file(
        package_name="super_odometry",
        file_name="config/tartan_air/tartan_air_calibration.yaml",
    )
    frame_normalizer_config = get_share_file(
        package_name="super_odometry",
        file_name="config/frame_normalizer_tartan_air.yaml",
    )
    home_directory = os.path.expanduser("~")

    config_path_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to tartan air config file for super_odometry",
    )
    calib_path_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib_path,
    )
    enable_frame_normalizer_arg = DeclareLaunchArgument(
        "enable_frame_normalizer",
        default_value="false",
        description="Enable frame_normalizer_node (optional for colorization/aligned outputs)",
    )
    enable_visual_odometry_arg = DeclareLaunchArgument(
        "enable_visual_odometry",
        default_value="true",
        description="Enable visual_odometry_node",
    )
    enable_visual_fusion_arg = DeclareLaunchArgument(
        "enable_visual_fusion",
        default_value="false",
        description="Enable visual fusion in laser_mapping_node",
    )
    enable_visual_deskew_arg = DeclareLaunchArgument(
        "enable_visual_deskew",
        default_value="false",
        description="Enable visual deskew path in feature_extraction_node",
    )

    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
                "feature_extraction_node.use_visual_deskew": LaunchConfiguration("enable_visual_deskew"),
            },
        ],
    )

    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
                "map_dir": os.path.join(home_directory, "/path/to/your/pcd"),
                "laser_mapping_node.enable_visual_fusion": LaunchConfiguration("enable_visual_fusion"),
            },
        ],
    )

    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
            },
        ],
    )

    visual_odometry_node = Node(
        package="super_odometry",
        executable="visual_odometry_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[LaunchConfiguration("config_file")],
        condition=IfCondition(LaunchConfiguration("enable_visual_odometry")),
    )

    frame_normalizer_node = Node(
        package="super_odometry",
        executable="frame_normalizer_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[frame_normalizer_config],
        condition=IfCondition(LaunchConfiguration("enable_frame_normalizer")),
    )

    return LaunchDescription(
        [
            launch_ros.actions.SetParameter(name="use_sim_time", value="true"),
            config_path_arg,
            calib_path_arg,
            enable_frame_normalizer_arg,
            enable_visual_odometry_arg,
            enable_visual_fusion_arg,
            enable_visual_deskew_arg,
            feature_extraction_node,
            laser_mapping_node,
            imu_preintegration_node,
            visual_odometry_node,
            frame_normalizer_node,
        ]
    )
