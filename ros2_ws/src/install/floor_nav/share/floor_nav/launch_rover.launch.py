import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node', 
            name='joy',
            parameters=[
                {'autorepeat_rate': 10.0},
                {'dev': "/dev/input/js0"},
            ],
            output='screen'
        ),

        launch_ros.actions.Node(
            package='topic_tools', 
            executable='mux', 
            name='cmd_mux',
            parameters=[
                {'output_topic': '/velocity_smoother/input'},
                {'input_topics': ['/teleop/twistCommand', '/mux/autoCommand']},
            ],
            output='screen'
        ),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', 
            executable='teleop_node', 
            name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 0.2},
                {'~/scale_angular': 1.0},
                {'~/timeout': 1.0}
            ],
            remappings=[
                ('twistCommand', '/teleop/twistCommand'),
            ],
            output='screen'
        ),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', 
            executable='teleop_mux_node', 
            name='teleop_mux',
            parameters=[
                {'~/joystick_button': 0},
                {'~/joystick_topic': '/teleop/twistCommand'},
                {'~/auto_button': 1},
                {'~/auto_topic': '/mux/autoCommand'}
            ],
            remappings=[
                ('select', '/cmd_mux/select'),
            ],
            output='screen'
        ),

        launch_ros.actions.Node(
            package='rover_driver_base', executable='rover_command_node', name='rover_command',
            parameters=[
                {'~/rover_name': 'rover'},
                {'~/skidsteer': False},
                {'~/check_timeout': False},
                ],
            remappings=[
                ('~/twistCommand', '/rover/twistCommand'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='floor_nav', 
            executable='floornav_task_server', 
            name='floor_tasks',
            parameters=[
                {'lib_path': os.path.join(os.getenv("HOME"), "ros2_ws/install/floor_nav/lib/floor_nav")},
                {'base_frame': 'base_link'},
                {'reference_frame': 'map'},
            ],
            remappings=[
                ('~/scans', '/scan'),
                ('/mux/autoCommand', '/mux/safeCommand'),
            ],
            output='screen'
        ),
    ])