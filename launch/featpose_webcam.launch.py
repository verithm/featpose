from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("featpose"))
        / "config"
        / "featpose_webcam.yaml"
    )
    rviz_config = str(
        Path(get_package_share_directory("featpose")) / "rviz" / "featpose.rviz"
    )

    return LaunchDescription(
        [
            Node(
                package="featpose",
                executable="featpose_webcam",
                name="featpose_publisher",
                parameters=[config],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-t", "FeatPose - RViz", "-d", rviz_config],
            ),
        ]
    )
