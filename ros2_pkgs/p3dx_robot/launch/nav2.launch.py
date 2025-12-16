from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('p3dx_robot')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # -------------------------------
        # Planner
        # -------------------------------
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        # -------------------------------
        # Controller
        # -------------------------------
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/cmd_vel', '/cmd_vel_nav'),
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static')],
        ),

        # -------------------------------
        # Smoother
        # -------------------------------
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        # -------------------------------
        # Behavior Tree Navigator
        # -------------------------------
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        # -------------------------------
        # Behavior Server
        # -------------------------------
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        # -------------------------------
        # Waypoint Follower
        # -------------------------------
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
        ),

        # -------------------------------
        # Velocity smoother
        # -------------------------------
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
        ),

        # -------------------------------
        # Lifecycle manager
        # -------------------------------
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'smoother_server',
                    'bt_navigator',
                    'behavior_server',
                    'waypoint_follower',
                    'velocity_smoother'
                ]
            }],
        ),
    ])

