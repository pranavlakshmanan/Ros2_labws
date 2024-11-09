import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.clock import Clock
import sys

class UR5TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('ur5_trajectory_node')

        topic_ = "/joint_states"
        self.joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Handle command-line arguments
        if len(sys.argv) < 7:
            self.get_logger().error("Not enough arguments provided. Using default values.")
            self.goal_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default values
        else:
            try:
                self.goal_ = [float(sys.argv[i]) for i in range(1, 7)]
            except ValueError:
                self.get_logger().error("Invalid argument(s) provided. Using default values.")
                self.goal_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default values

        self.publisher_ = self.create_publisher(JointState, topic_, 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        current_time = self.get_clock().now().to_msg()

        msg.header.stamp = current_time
        msg.name = self.joints
        msg.position = self.goal_

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing positions: {self.goal_}")

def main(args=None):
    rclpy.init(args=args)
    node = UR5TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()