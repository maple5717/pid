#! /usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
# from math import abs

# TODO: if expected_vel not received in a long time, set output 0
# TODO: WHy robot stops after the target vel reached immediately?
# TODO: stop the robot once vio is lost
class VelocityControllerNode(Node):
    def __init__(self):
        super().__init__('velocity_controller')


        # Publishers
        self.x_vel_publisher = self.create_publisher(Float64, '/pid_vel/x_lin_expected', 10)
        self.z_vel_publisher = self.create_publisher(Float64, '/pid_vel/z_rot_expected', 10)
        self.x_obs_publisher = self.create_publisher(Float64, '/pid_vel/x_lin_obs', 10)
        self.z_obs_publisher = self.create_publisher(Float64, '/pid_vel/z_rot_obs', 10)
        self.x_enable_publisher = self.create_publisher(Bool, '/pid_vel/x_lin_enable', 10)
        self.z_rot_enable_publisher = self.create_publisher(Bool, '/pid_vel/z_rot_enable', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.sub1 = self.create_subscription(Twist, '/expected_vel', self.expected_vel_callback, 1)
        self.sub2 = self.create_subscription(Odometry, '/transformed_odom', self.odometry_callback, 1)  # Subscribe to odometry
        self.sub3 = self.create_subscription(Float64, '/pid_vel/x_lin_cmd', self.x_cmd_callback, 1)  # Subscribe to x_cmd
        self.sub4 = self.create_subscription(Float64, '/pid_vel/z_rot_cmd', self.z_cmd_callback, 1)  # Subscribe to z_cmd

        # Initialize commands
        self.x_lin_cmd = 0.0
        self.z_rot_cmd = 0.0
        self.x_lin_expected = 0.0
        self.z_rot_expected = 0.0

        # Create timer to periodically send cmd_vel
        self.create_timer(0.05, self.publish_cmd_vel)

        self.rot_threshold = 1e-5

        self.get_logger().info("PID helper started for the wheelchair!")

    def expected_vel_callback(self, msg: Twist):
        
        # Extract linear x and rotational z velocities
        linear_x = msg.linear.x
        rotational_z = msg.angular.z

        self.z_rot_expected = rotational_z

        # Publish the velocities
        self.x_vel_publisher.publish(Float64(data=linear_x))
        self.z_vel_publisher.publish(Float64(data=rotational_z))

        # self.get_logger().info(rotational_z)
        z_enable_msg = Bool()
        z_enable_msg.data = True if (abs(rotational_z) > self.rot_threshold) else False
        self.z_rot_enable_publisher.publish(z_enable_msg)


    def x_cmd_callback(self, msg: Float64):
        # Update x_cmd from the received message
        self.x_lin_cmd = msg.data

    def z_cmd_callback(self, msg: Float64):
        # Update z_cmd from the received message
        self.z_rot_cmd = msg.data # + self.z_rot_expected # ffw control

    def odometry_callback(self, msg: Odometry):
        # Extract linear x and rotational z velocities from odometry
        self.x_lin_obs = msg.twist.twist.linear.x
        self.z_rot_obs = msg.twist.twist.angular.z

        # Publish the odometry values as separate messages
        self.x_obs_publisher.publish(Float64(data=self.x_lin_obs))
        self.z_obs_publisher.publish(Float64(data=self.z_rot_obs))

        

    def publish_cmd_vel(self):
        # Create Twist message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3(x=self.x_lin_cmd, y=0.0, z=0.0)
        cmd_vel_msg.angular = Vector3(x=0.0, y=0.0, z=self.z_rot_cmd)

        # Publish cmd_vel
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    print("exit")

if __name__ == '__main__':
    main()
