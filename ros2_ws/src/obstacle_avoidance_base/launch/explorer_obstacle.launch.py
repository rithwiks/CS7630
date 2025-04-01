import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[
                {'autorepeat_rate': 10.},
                {'dev': "/dev/input/js0"},
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='topic_tools', executable='mux', name='cmd_mux',
            parameters=[
                {'output_topic': '/vrep/twistCommand'},
                {'input_topics': ['/teleop/twistCommand','/mux/autoCommand']},
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 0.2},
                {'~/scale_angular': 1.},
                {'~/timeout': 1.0}
            ],
            remappings=[
                ('twistCommand', '/teleop/twistCommand'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_mux',
            parameters=[
                {'~/joystick_button': 0},
                {'~/joystick_topic': '/teleop/twistCommand'},
                {'~/auto_button': 1},
                {'~/auto_topic': '/mux/autoCommand'}
            ],
            remappings=[
                ('select', '/cmd_mux/select'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_explorer', executable='occgrid_planner', name='occgrid_planner',
            parameters=[
                {'~/neighbourhood': 8},
                {'~/base_frame': 'bubbleRob'},
                {'~/debug': False},
                {'~/headless': False},
            ],
            remappings=[
                ('~/occ_grid', '/map'),
                ('~/goal', '/goal_pose'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_explorer', executable='path_optimizer', name='path_optimizer',
            parameters=[
                {'~/max_acceleration': 0.3},
                {'~/max_braking': 0.1},
                {'~/velocity': .25},
            ],
            remappings=[
                ('~/path', '/occgrid_planner/path'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='obstacle_avoidance_base', executable='obstacle_avoidance_base', name='obstacle_avoidance',
            parameters=[
                {'~/base_frame': 'bubbleRob'},
                {'~/display': False},
                {"~/max_range": 5.0},
                {"~/max_linear_velocity": 1.0},
                {"~/max_angular_velocity": 1.0},
                {"~/max_linear_accel": 5.0},
                {"~/max_angular_accel": 5.0},
                {"~/map_resolution": 0.05},
                {"~/linear_velocity_resolution": 0.05},
                {"~/angular_velocity_resolution": 0.05},
                {"~/robot_radius": 0.1},
                {"~/time_horizon": 2.0},
                {"~/k_v": 10.0},
                {"~/k_w": 1.0},
            ],
            remappings=[
                ('~/scans', '/vrep/hokuyo'),
                ('~/current_velocity', '/vrep/twistStatus'),
                ('~/command_velocity', '/obstacle_avoidance/command_velocity'),
                ('~/odom', '/odom'),
                ('~/output_velocity', '/mux/autoCommand'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_explorer', executable='path_follower', name='path_follower',
            parameters=[
                {'~/Kx': 1.0},
                {'~/Ky': 0.0},
                {'~/Ktheta': 1.0},
                {'~/max_rot_speed': 1.0},
                {'~/max_velocity': 0.25},
                {'~/max_y_error': 1.0},
                {'~/max_error': 0.5},
                {'~/look_ahead': 1.0},
                {'~/base_frame': 'bubbleRob'},
            ],
            remappings=[
                ('~/traj', '/path_optimizer/trajectory'),
                ('~/twistCommand', '/obstacle_avoidance/command_velocity'),
                ('~/goal', '/goal_pose'),
                ('~/obstacle_avoidance', '/obstacle_avoidance/command_velocity'),
            ],
            output='screen'),
    ])
