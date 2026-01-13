import os
from tkinter import NO
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    package_name = 'pointcloud2_slam'
    share_dir = get_package_share_directory(package_name)

    point_transform_node = Node(
        package='pointcloud2_slam',
        executable='point_transform_node',
        name='point_transform_node',
        output='screen',
    )

    obstacle_grid_node = Node(
        package='pointcloud2_slam',
        executable='obstacle_grid_node',
        name='obstacle_grid_node',
        output='screen',
    )

    astar = Node(
        package='pointcloud2_slam',
        executable='astar',
        name='astar',
        output='screen',
    )

    start_nav = Node(
        package='pointcloud2_slam',
        executable='start_nav',
        name='start_nav',
        output='screen',
    )

    odom_map_tf = Node(
        package='pointcloud2_slam',
        executable='odom_map_tf',
        name='odom_map_tf',
        output='screen',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    ld.add_action(point_transform_node)
    ld.add_action(obstacle_grid_node)
    ld.add_action(astar)
    ld.add_action(start_nav)
    ld.add_action(odom_map_tf)
    ld.add_action(rviz2_node)


    return ld