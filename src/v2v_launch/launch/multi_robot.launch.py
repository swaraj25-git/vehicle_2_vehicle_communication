#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    ros_gz_sim_dir = get_package_share_directory(
        'ros_gz_sim'
    )

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------

    model_path = os.path.join(
        turtlebot3_gazebo_dir,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    world_path = os.path.join(
        turtlebot3_gazebo_dir,
        'worlds',
        'turtlebot3_world.world'
    )

    bridge_config = os.path.join(
        turtlebot3_gazebo_dir,
        'params',
        'turtlebot3_burger_bridge.yaml'
    )

    # ---------------------------------------------------------
    # Gazebo Harmonic
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v2 {world_path}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ---------------------------------------------------------
    # Robot 1
    # ---------------------------------------------------------

    robot1_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot1',
        name='spawn_robot1',
        arguments=[
            '-name', 'robot1',
            '-file', model_path,
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.01'
        ],
        output='screen'
    )

    robot1_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='robot1',
        name='bridge',
        parameters=[
            {'config_file': bridge_config}
        ],
        output='screen'
    )

    robot1_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        name='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': open(
                    os.path.join(
                        turtlebot3_gazebo_dir,
                        'urdf',
                        'turtlebot3_burger.urdf'
                    )
                ).read(),
                'frame_prefix': 'robot1/'
            }
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Robot 2
    # ---------------------------------------------------------

    robot2_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot2',
        name='spawn_robot2',
        arguments=[
            '-name', 'robot2',
            '-file', model_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    robot2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='robot2',
        name='bridge',
        parameters=[
            {'config_file': bridge_config}
        ],
        output='screen'
    )

    robot2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        name='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': open(
                    os.path.join(
                        turtlebot3_gazebo_dir,
                        'urdf',
                        'turtlebot3_burger.urdf'
                    )
                ).read(),
                'frame_prefix': 'robot2/'
            }
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Robot 3
    # ---------------------------------------------------------

    robot3_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot3',
        name='spawn_robot3',
        arguments=[
            '-name', 'robot3',
            '-file', model_path,
            '-x', '2.0',
            '-y', '0.5',
            '-z', '0.01'
        ],
        output='screen'
    )

    robot3_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='robot3',
        name='bridge',
        parameters=[
            {'config_file': bridge_config}
        ],
        output='screen'
    )

    robot3_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot3',
        name='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': open(
                    os.path.join(
                        turtlebot3_gazebo_dir,
                        'urdf',
                        'turtlebot3_burger.urdf'
                    )
                ).read(),
                'frame_prefix': 'robot3/'
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,

        robot1_spawn,
        robot1_bridge,
        robot1_rsp,

        robot2_spawn,
        robot2_bridge,
        robot2_rsp,

        robot3_spawn,
        robot3_bridge,
        robot3_rsp,
    ])