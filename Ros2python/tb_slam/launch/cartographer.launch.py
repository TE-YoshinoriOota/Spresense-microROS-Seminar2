#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    project_dir = get_package_share_directory('tb_slam')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_tf = LaunchConfiguration('enable_tf', default='false')
    odom_tf = LaunchConfiguration('odom_tf', default='false')

    #
    # launch cartographer
    #
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
                    default=os.path.join(project_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                    default='lds_2d.lua')
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename',  configuration_basename]        
    )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    #
    # display rivz2
    #
    rviz_config_dir = os.path.join(project_dir, 'rviz', 'cartographer.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz2_node)

    return ld


