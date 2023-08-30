#!/usr/bin/env python3

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = get_package_share_directory('tb_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    ) 
    
    result = False
    for s in sys.argv:
        if 'odom_tf' in s and (('True' in s or 'true' in s)):
                result = True

    if result:
        urdf = os.path.join(project_dir, 'urdf', 'tamiyan_odom.urdf')
    else:
        urdf = os.path.join(project_dir, 'urdf', 'tamiyan.urdf')

    # print('urdf file is' + urdf)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tamiyan',
            '-file', urdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    ld.add_action(robot_state)
    ld.add_action(spawn_entity)

    return ld

