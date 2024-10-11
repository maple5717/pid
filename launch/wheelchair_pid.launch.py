import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid',               # Name of your package
            executable='controller',     # Name of your executable
            name='steering_control',          # Node name
            output='screen',             # Output to screen
            parameters=[
                {'Kp': 0.03},
                {'Ki': 0.06},
                {'Kd': 0.001},
                {'upper_limit': 15.0},
                {'lower_limit': -15.0},
                {'windup_limit': 25.0},
                {'cutoff_frequency': 20.0},
                {'max_loop_frequency': 105.0},
                {'min_loop_frequency': 95.0},
                {'setpoint_timeout': -1.0},
            ]
        ),
    ])
