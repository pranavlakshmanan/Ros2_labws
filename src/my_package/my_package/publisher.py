#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
class RobotPublisher(Node):

    def __init__(self):
        super().__init__("publisher")
        self.robot_name_="COBOT"
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Publisher Node Started")


    def publish_news(self):
        msg = String()
        msg.data = "Greetings from " + str(self.robot_name_)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()