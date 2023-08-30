import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('ransac'),
        'param',
        'ransac.yaml'
    )

    ransac_rviz_config = os.path.join(
        get_package_share_directory('ransac'),
        'rviz',
        'ransac2.rviz'
    )
    
    ransac_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', ransac_rviz_config],
    )

    ransac_node = Node(
        package='ransac',
        executable='ransac_node',
        namespace='ransac_param',
        name='param',
        parameters=[param],
    )

    return LaunchDescription([
        ransac_node,
        ransac_rviz_node,
    ])