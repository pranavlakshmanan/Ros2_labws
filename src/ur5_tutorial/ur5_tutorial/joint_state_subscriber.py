import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
       def __init__(self):
           super().__init__('joint_state_subscriber')
           self.subscription = self.create_subscription(
               JointState,
               '/joint_states',
               self.listener_callback,
               10)

       def listener_callback(self, msg):
           self.get_logger().info(f"Received joint states: {msg.position}")

def main(args=None):
       rclpy.init(args=args)
       joint_state_subscriber = JointStateSubscriber()
       rclpy.spin(joint_state_subscriber)
       joint_state_subscriber.destroy_node()
       rclpy.shutdown()

if __name__ == '__main__':
       main()
