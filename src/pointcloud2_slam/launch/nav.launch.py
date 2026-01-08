import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    package_name = 'pointcloud2_slam'
    share_dir = get_package_share_directory(package_name)

    point_transform_node = Node( # 发布优化后的点云
        package='pointcloud2_slam',
        executable='point_transform_node',
        name='point_transform_node',
        output='screen',
    )

    obstacle_grid_node = Node( # 基于优化后的点构建2d地图/lio_sam/mapping/cloud_registered
        package='pointcloud2_slam',
        executable='obstacle_grid_node',
        name='obstacle_grid_node',
        output='screen',
    )

    ld.add_action(point_transform_node)
    ld.add_action(obstacle_grid_node)

    return ld