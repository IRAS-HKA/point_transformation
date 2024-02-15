import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('point_transformation'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='point_transformation',
            executable='point_transformation_node',
            name='point_transformation_node',
            output='screen',
            parameters=[config],
            prefix=['stdbuf -o L'],
        )
    ])
