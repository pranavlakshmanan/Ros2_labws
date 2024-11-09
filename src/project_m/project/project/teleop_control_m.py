#!/usr/bin/env python3

import os
import sys
import select  # Changed from: from numpy import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
import termios
import tty

MSG = """
Control Your Mecanum Robots!
---------------------------
Robot1 (Keyboard Controls):
        w     
   a    s    d    
        x     

w/x : move forward/backward
a/d : move left/right
q/e : rotate left/right
s   : force stop
---------------------------
Robot2 (Joystick Controls):
Left Stick Y-axis  : forward/backward
Left Stick X-axis  : left/right
Right Stick X-axis : rotation
A Button           : emergency stop
---------------------------
CTRL-C to quit
"""

class MecanumTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('mecanum_teleop_node')
        
        # Initialize terminal settings
        self.settings = None
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        
        # Publishers
        self.cmd_vel_pub1 = self.create_publisher(Twist, '/Robot1/cmd_vel', 10)
        self.cmd_vel_pub2 = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)
        
        # Joystick subscriber for Robot2
        self.create_subscription(Joy, '/Robot2/joy', self.joy_callback, 10)
        
        # Velocity parameters
        self.linear_vel = 0.5
        self.angular_vel = 1.57
        self.vel_step = 0.1

        # Start keyboard control timer
        self.create_timer(0.1, self.keyboard_control)

    def get_key(self):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_control(self):
        """Handle keyboard inputs for Robot1"""
        key = self.get_key()
        twist = Twist()
        
        if key == 'w':
            twist.linear.x = self.linear_vel
        elif key == 'x':
            twist.linear.x = -self.linear_vel
        elif key == 'a':
            twist.linear.y = self.linear_vel
        elif key == 'd':
            twist.linear.y = -self.linear_vel
        elif key == 'q':
            twist.angular.z = self.angular_vel
        elif key == 'e':
            twist.angular.z = -self.angular_vel
        elif key == 's':
            # Emergency stop
            twist = Twist()
        
        self.cmd_vel_pub1.publish(twist)
        
        if key == '\x03':
            self.shutdown()

    def joy_callback(self, msg: Joy):
        """Handle joystick inputs for Robot2"""
        twist = Twist()
        
        # Linear motion
        twist.linear.x = msg.axes[1] * self.linear_vel  # Left stick Y-axis
        twist.linear.y = msg.axes[0] * self.linear_vel  # Left stick X-axis
        
        # Angular motion
        twist.angular.z = msg.axes[3] * self.angular_vel  # Right stick X-axis
        
        # Emergency stop if A button (button[0]) is pressed
        if msg.buttons[0]:
            twist = Twist()
            
        self.cmd_vel_pub2.publish(twist)

    def shutdown(self):
        """Clean shutdown of both robots"""
        stop_twist = Twist()
        self.cmd_vel_pub1.publish(stop_twist)
        self.cmd_vel_pub2.publish(stop_twist)
        # Restore terminal settings before exit
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = MecanumTeleopNode()
        print(MSG)
        rclpy.spin(teleop_node)
    except Exception as e:
        print(e)
    finally:
        if hasattr(teleop_node, 'settings') and teleop_node.settings and os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop_node.settings)
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()