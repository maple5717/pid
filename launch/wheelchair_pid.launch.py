from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid',               # Name of your package
            executable='controller',     # Name of your executable
            name='angular_velocity_control',     # Node name
            output='screen',             # Output to screen
            parameters=[
                {'Kp': 2.0}, # 0.8
                {'Ki': 20.0}, # 10.0
                {'Kd': 0.000}, # 0.0
                {'windup_limit': 0.025}, # 0.05 # TODO: see why y!=0 for cmd_vel_nav
                {'upper_limit': 1.20-0.4},
                {'lower_limit': -1.20+0.4},
                {'cutoff_frequency': 20.0},
                {'max_loop_frequency': 105.0},
                {'min_loop_frequency': 10.0},
                {'setpoint_timeout': 0.1},
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
            executable='controller',     # Name of your executable
            name='linear_velocity_control',     # Node name
            output='screen',             # Output to screen
            parameters=[
                {'Kp': 0.5},
                {'Ki': 1.0}, # 2.0
                {'Kd': 0.0}, # 1.0
                {'windup_limit': 0.16},
                {'upper_limit': 0.2},
                {'lower_limit': -0.2},
                {'cutoff_frequency': 20.0},
                {'max_loop_frequency': 105.0},
                {'min_loop_frequency': 10.0},
                {'setpoint_timeout': 0.1},
                {'topic_from_controller': '/pid_vel/x_lin_cmd'},
                {'topic_from_plant': '/pid_vel/x_lin_obs'},
                {'setpoint_topic': '/pid_vel/x_lin_expected'},
                {'pid_enable_topic': '/pid_vel/x_lin_enable'},
                {'pid_debug_topic': '/pid_vel/x_lin_debug'},
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
