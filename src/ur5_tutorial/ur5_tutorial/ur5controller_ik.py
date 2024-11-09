import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.clock import Clock
import sys
import atexit
import math

class InverseKinematicsPublisher(Node):
    def __init__(self):
        super().__init__('ik_trajectory_node')

        topic_ = "/joint_states"
        self.joints = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 
            'elbow_joint', 'wrist_1_joint', 
            'wrist_2_joint', 'wrist_3_joint'
        ]

        # Handle command-line arguments
        if len(sys.argv) < 7:
            self.get_logger().error("Not enough arguments provided. Using default values.")
            self.goal_pose = [0.3, -0.5, 0.6, -1.2, 0.8, -0.4]  # Default values
        else:
            try:
                self.goal_pose = [float(sys.argv[i]) for i in range(1, 7)]
            except ValueError:
                self.get_logger().error("Invalid argument(s) provided. Using default values.")
                self.goal_pose = [0.3, -0.5, 0.6, -1.2, 0.8, -0.4]  # Default values

        self.publisher_ = self.create_publisher(JointState, topic_, 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # For interpolation
        self.current_position = [0.0] * len(self.joints)
        self.step_size = 0.01
        self.goal_reached = False

        # UR5 link lengths
        self.a = [0, -0.425, -0.39225, 0, 0, 0]
        self.d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]

        # Register exit handler
        atexit.register(self.hold_final_position)

    def inverse_kinematics(self, pose):
        x, y, z, rx, ry, rz = pose
        theta = [0] * 6

        # Solve for theta1
        theta[0] = math.atan2(y, x) - math.atan2(self.d[3], math.sqrt(x*x + y*y - self.d[3]*self.d[3]))

        # Solve for theta5
        c1 = math.cos(theta[0])
        s1 = math.sin(theta[0])
        c5 = (z - self.d[0] - self.d[4]*(c1*math.cos(ry) + s1*math.sin(rx)*math.sin(ry)) + 
              self.d[5]*(c1*math.sin(ry) - s1*math.sin(rx)*math.cos(ry))) / self.d[5]
        c5 = max(min(c5, 1), -1)  # Clamp c5 to the range [-1, 1]
        theta[4] = math.acos(c5)

        # Solve for theta6
        s5 = math.sin(theta[4])
        theta[5] = math.atan2((-s1*math.cos(rx)*math.sin(ry) + c1*math.cos(ry))/s5, 
                              (s1*math.cos(rx)*math.cos(ry) + c1*math.sin(ry))/s5)

        # Solve for theta3
        c234 = (z - self.d[0] - self.d[4]*math.sin(theta[4])) / (self.a[1] + self.a[2])
        s234 = math.sqrt(1 - c234*c234)
        theta[2] = math.atan2(s234, c234) - theta[1] - theta[0]

        # Solve for theta2
        r = math.sqrt(x*x + y*y)
        s2 = ((self.a[1] + self.a[2]*c234)*r - (self.a[2]*s234 + self.d[3])*(z - self.d[0])) / (self.a[1]*self.a[1] + self.a[2]*self.a[2])
        c2 = ((self.a[2]*s234 + self.d[3])*r + (self.a[1] + self.a[2]*c234)*(z - self.d[0])) / (self.a[1]*self.a[1] + self.a[2]*self.a[2])
        theta[1] = math.atan2(s2, c2)

        # Solve for theta4
        theta[3] = ry - theta[1] - theta[2]

        return theta

    def timer_callback(self):
        msg = JointState()
        current_time = Clock().now().to_msg()

        msg.header.stamp.sec = current_time.sec
        msg.header.stamp.nanosec = current_time.nanosec
        msg.name = self.joints

        if not self.goal_reached:
            goal_joint_angles = self.inverse_kinematics(self.goal_pose)
            
            # Interpolate each joint position towards the goal
            for i in range(len(self.joints)):
                self.current_position[i] += (goal_joint_angles[i] - self.current_position[i]) * self.step_size

            # Check if the goal is reached
            if all(abs(goal_joint_angles[i] - self.current_position[i]) < 0.01 for i in range(len(self.joints))):
                self.goal_reached = True
                self.get_logger().info("Goal reached. Holding position...")

        msg.position = self.current_position
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing position: {}".format(self.current_position))

    def hold_final_position(self):
        msg = JointState()
        current_time = Clock().now().to_msg()

        msg.header.stamp.sec = current_time.sec
        msg.header.stamp.nanosec = current_time.nanosec
        msg.name = self.joints
        msg.position = self.current_position

        # Publish the final position one more time to ensure it's held
        self.publisher_.publish(msg)
        self.get_logger().info("Final position held: {}".format(self.current_position))

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()