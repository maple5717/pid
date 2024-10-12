from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid',               # Name of your package
            executable='controller',     # Name of your executable
            name='steering_control',     # Node name
            output='screen',             # Output to screen
            parameters=[
                {'Kp': 2.5},
                {'Ki': 1.0}, # 2.0
                {'Kd': 0.000}, # 1.0
                {'upper_limit': 1.20-0.4},
                {'lower_limit': -1.20+0.4},
                {'windup_limit': 1.2},
                {'cutoff_frequency': 20.0},
                {'max_loop_frequency': 105.0},
                {'min_loop_frequency': 10.0},
                {'setpoint_timeout': -1.0},
                {'topic_from_controller': '/pid_vel/z_rot_cmd'},
                {'topic_from_plant': '/pid_vel/z_rot_obs'},
                {'setpoint_topic': '/pid_vel/z_rot_expected'},
                {'pid_enable_topic': '/pid_vel/z_rot_enable'},
                {'pid_debug_topic': '/pid_vel/z_rot_debug'},
                {'angle_error': False},
                # {'angle_wrap': 2.0 * 3.14159},  # Using a numerical expression
            ]
        ),

         Node(
            package='pid',               # Name of your package
            executable='wheelchair_pid_helper.py',     # Name of your executable
            name='pid_helper',     # Node name
            output='screen',
         )
    ])
