#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class MecanumController(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_mecanum_controller')
        
        # Create velocity command subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/{robot_name}/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Create wheel velocity publisher
        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            f'/{robot_name}/forward_velocity_controller/commands',
            10)
            
        # Wheel geometry parameters (adjust these based on your URDF)
        self.wheel_radius = 0.075
        self.wheel_separation_width = 0.51834  # distance between left and right wheels
        self.wheel_separation_length = 1.06667 # distance between front and back wheels
        
        self.get_logger().info(f'Initialized mecanum controller for {robot_name}')
        
        # Create timer for periodic updates
        self.create_timer(0.1, self.timer_callback)  # 10Hz control loop
        
    def timer_callback(self):
        # Publish zero velocity if no commands received
        cmd = Twist()
        self.cmd_vel_callback(cmd)
        
    def cmd_vel_callback(self, msg):
        # Extract commanded velocities
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Debug output
        self.get_logger().debug(f'Received cmd_vel - vx: {vx}, vy: {vy}, wz: {wz}')
        
        # Mecanum wheel kinematics
        # Wheel velocities = [front_left, front_right, rear_left, rear_right]
        wheel_velocities = [
            (vx - vy - wz * (self.wheel_separation_width + self.wheel_separation_length) / 2.0) / self.wheel_radius,
            (vx + vy + wz * (self.wheel_separation_width + self.wheel_separation_length) / 2.0) / self.wheel_radius,
            (vx + vy - wz * (self.wheel_separation_width + self.wheel_separation_length) / 2.0) / self.wheel_radius,
            (vx - vy + wz * (self.wheel_separation_width + self.wheel_separation_length) / 2.0) / self.wheel_radius
        ]
        
        # Create and publish wheel velocity command
        vel_cmd = Float64MultiArray()
        vel_cmd.data = wheel_velocities
        self.vel_pub.publish(vel_cmd)
        
        # Debug output
        if any(abs(v) > 0.01 for v in wheel_velocities):
            self.get_logger().info(f'Publishing wheel velocities: {[round(v,3) for v in wheel_velocities]}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create controllers for both robots
    controller1 = MecanumController('robot_1')
    controller2 = MecanumController('robot_2')
    
    # Use MultiThreadedExecutor to run both controllers
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller1)
    executor.add_node(controller2)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        controller1.destroy_node()
        controller2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()