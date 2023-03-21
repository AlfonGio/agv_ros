import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    box_filter_params = os.path.join(get_package_share_directory('agv_ros'),'config','box_laser_filter.yaml')
    box_filter = Node(package="laser_filters", executable="scan_to_scan_filter_chain",
                      parameters=[box_filter_params],
        )

    return LaunchDescription([
        box_filter
    ])
