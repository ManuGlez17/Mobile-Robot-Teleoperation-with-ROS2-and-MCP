import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    package_name = 'p3dx_robot'
  
    rviz_config_file = os.path.join(
    get_package_share_directory(package_name),
    'config',
    'view_bot.rviz'
    )  
    
    # Launch robot_state_publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    

    # Launch Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Spawn robot into Gazebo (wait 5s)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                output='screen'
            )
        ]
    )

    # Joint State Broadcaster (wait 7s)
    joint_broad_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
                output="screen"
            )
        ]
    )

    # Diff drive controller (wait 8s)
    diff_drive_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
                output="screen"
            )
        ]
    )
    
    rviz_node = TimerAction(
    period=8.0,
    actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        )
    ]
    )
    
    
    

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        joint_broad_spawner,
        diff_drive_spawner,
        rviz_node
    ])

