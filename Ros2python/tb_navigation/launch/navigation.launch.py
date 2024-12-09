import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = get_package_share_directory('tb_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    odom_tf = LaunchConfiguration('odom_tf', default='false')

    #
    #  launch navigation2
    #
    map_file = LaunchConfiguration(
        'map', 
        default=os.path.join(project_dir, 'map', 'map.yaml')        
    )

    param_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(project_dir, 'param', 'tamiyan.yaml')
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file_dir = os.path.join(nav2_bringup_dir, 'launch')
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': param_file}.items(),
    )

    #
    #  launch rviz2
    #
    rviz_config_dir = os.path.join(project_dir, 'rviz', 'nav2.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    ) 


    # ekf_config_dir = os.path.join(project_dir, 'ekf', 'ekf_config.yaml')
    # localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config_dir,{'use_sim_time': use_sim_time}],
    #     # remappings=[('/odometry/filtered', '/odom'),]
    # )

    ld = LaunchDescription()
    ld.add_action(bringup_launch)
    ld.add_action(rviz2_node)
    # ld.add_action(localization_node)

    return ld
