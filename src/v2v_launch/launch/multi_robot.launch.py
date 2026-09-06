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
    v2v_launch_dir = get_package_share_directory('v2v_launch')

    # Paths
    # ---------------------------------------------------------

    robot1_model_path = os.path.join(
    v2v_launch_dir,
    'models',
    'robot1.sdf'
    )

    robot2_model_path = os.path.join(
        v2v_launch_dir,
        'models',
        'robot2.sdf'
    )

    robot3_model_path = os.path.join(
        v2v_launch_dir,
        'models',
        'robot3.sdf'
    )

    world_path = os.path.join(
        turtlebot3_gazebo_dir,
        'worlds',
        'turtlebot3_world.world'
    )

    robot1_bridge_config = os.path.join(
    v2v_launch_dir,
    'config',
    'robot1_bridge.yaml'
)

    robot2_bridge_config = os.path.join(
        v2v_launch_dir,
        'config',
        'robot2_bridge.yaml'
    )

    robot3_bridge_config = os.path.join(
        v2v_launch_dir,
        'config',
        'robot3_bridge.yaml'
    )

    #gazebo

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


    # Robot 1


    robot1_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot1',
        name='spawn_robot1',
        arguments=[
            '-name', 'robot1',
            '-file', robot1_model_path,
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
            {'config_file': robot1_bridge_config}
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

   

    robot2_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot2',
        name='spawn_robot2',
        arguments=[
            '-name', 'robot2',
            '-file', robot2_model_path,
            '-x', '1.5',
            '-y', '0.5',
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
            {'config_file': robot2_bridge_config}
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



    robot3_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot3',
        name='spawn_robot3',
        arguments=[
            '-name', 'robot3',
            '-file', robot3_model_path,
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
            {'config_file': robot3_bridge_config}
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