#! /usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
# from math import abs

# Note: Currently PID is only used for rotation control!!! 
# TODO (done): if expected_vel not received in a long time, set output 0
# TODO: 1. determinet speed range 2. tune PID controller accordingly 3. check controller 
# TODO: if the expected output is 0, disable the pid controller
# TODO: stop the robot once vio is lost
# TODO: tune the filter to eliminate noise in D term so it can be used
# WHy robot stops after the target vel reached immediately? ans: Filter for the P component! 
class VelocityControllerNode(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.lin_min_th = 0.1
        self.rot_min_th = 0.1 

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
        self.x_lin_expected_prev = 0.0
        self.z_rot_expected_prev = 0.0
        self.x_lin_cmd = 0.0
        self.z_rot_cmd = 0.0
        self.x_lin_expected = 0.0
        self.z_rot_expected = 0.0
        self.last_cmd_time = self.get_clock().now() 

        # Create timer to periodically send cmd_vel
        self.create_timer(0.05, self.publish_cmd_vel)
        self.create_timer(0.1, self.watchdog_callback)

        self.rot_threshold = 1e-5

        self.get_logger().info("PID helper started for the wheelchair!")

    def expected_vel_callback(self, msg: Twist):
        
        # Extract linear x and rotational z velocities
        linear_x = msg.linear.x
        rotational_z = msg.angular.z
        
        # avoid outputing vel of small magnitude
        if abs(rotational_z) < self.rot_min_th and rotational_z != 0:
            rotational_z = self.rot_min_th * rotational_z / abs(rotational_z)

        if abs(linear_x) < self.lin_min_th and linear_x != 0:
            linear_x = self.lin_min_th * linear_x / abs(linear_x)

        # ugly! ROBOT MOVES EVEN THERE IS NO CONTROL INPUT! 
        # IF PREV IS NOT 0
        a = 0.9 * 0
        self.z_rot_expected = (1-a) * rotational_z + a * self.z_rot_expected_prev
        self.x_lin_expected = (1-a) * linear_x + a * (self.x_lin_expected_prev)

        self.z_rot_expected_prev = self.z_rot_expected
        self.x_lin_expected_prev = self.x_lin_expected

        # Publish the velocities
        self.x_vel_publisher.publish(Float64(data=linear_x))
        self.z_vel_publisher.publish(Float64(data=rotational_z))

        # self.get_logger().info(rotational_z)
        z_enable_msg = Bool()
        z_enable_msg.data = True if (abs(rotational_z) > self.rot_threshold) else False
        self.z_rot_enable_publisher.publish(z_enable_msg)

        self.last_cmd_time = self.get_clock().now()

    def x_cmd_callback(self, msg: Float64):
        # Update x_cmd from the received message
        self.x_lin_cmd = self.x_lin_expected # msg.data

        # prevent sudden stop of the robot
        if self.x_lin_cmd * self.x_lin_expected <= 0 and self.x_lin_expected != 0:
            self.x_lin_cmd = 0.05 * self.x_lin_expected / abs(self.x_lin_expected)

        if abs(self.x_lin_expected) < 1e-5:
            self.x_lin_cmd = 0.0
        

    def z_cmd_callback(self, msg: Float64):
        # Update z_cmd from the received message
        self.z_rot_cmd = msg.data + self.z_rot_expected # ffw control
        
        # prevent sudden stop of the robot
        # explain: Occasionally, overshooting can result in commands being sent in the opposite direction, 
        # causing the robot's brakes to engage and leading to unwanted stops.
        # This code prevents the robot from breaking
        if self.z_rot_cmd * self.z_rot_expected <= 0 and self.z_rot_expected != 0:
            self.z_rot_cmd = 0.05 * self.z_rot_expected / abs(self.z_rot_expected)

        # if abs(self.z_rot_expected) < 1e-5:
        #     self.z_rot_cmd = 0.0
            
        

    def odometry_callback(self, msg: Odometry):
        # Extract linear x and rotational z velocities from odometry
        self.x_lin_obs = msg.twist.twist.linear.x
        self.z_rot_obs = msg.twist.twist.angular.z

        # Publish the odometry values as separate messages
        self.x_obs_publisher.publish(Float64(data=self.x_lin_obs))
        self.z_obs_publisher.publish(Float64(data=self.z_rot_obs))

    def publish_cmd_vel(self):
        # self.get_logger().warn(f"{self.x_lin_cmd}  {self.z_rot_cmd}")
        # Create Twist message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3(x=self.x_lin_cmd, 
                                     y=0.0, z=0.0)
        cmd_vel_msg.angular = Vector3(x=0.0, y=0.0, z=self.z_rot_cmd)

        # Publish cmd_vel
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def watchdog_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_cmd_time).nanoseconds * 1e-9  # Convert to seconds

        if elapsed_time > 0.1:  # If more than 0.2 seconds have passed
            # self.get_logger().warn("No command received in the last 0.2 seconds. Stopping the robot.")
            self.x_lin_cmd = 0.0
            self.z_rot_cmd = 0.0  # Set to zero or handle accordingly
            self.x_lin_expected = 0.0
        

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    print("exit")

if __name__ == '__main__':
    main()
