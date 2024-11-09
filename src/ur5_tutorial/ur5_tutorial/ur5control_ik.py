#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from PyKDL import Chain, Segment, Joint, Frame, Vector, Rotation
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverVel_pinv, ChainIkSolverPos_NR_JL

class UR5InverseKinematicsController(Node):
    
    def __init__(self):
        super().__init__('ur5_ik_controller_node')
        self.joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.declare_parameter("end_effector_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.goal_pose = self.get_parameter("end_effector_pose").value
        self.publisher_ = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.timer_ = self.create_timer(1, self.timer_callback)
        
        # Create KDL Chain
        self.chain = self.create_kdl_chain()
        
        # Create KDL solvers
        self.fk_solver = ChainFkSolverPos_recursive(self.chain)
        self.ik_v_solver = ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = ChainIkSolverPos_NR_JL(self.chain, self.fk_solver, self.ik_v_solver, 100, 1e-6)
        
    def create_kdl_chain(self):
        chain = Chain()
        
        # Add segments to the chain (simplified, you may need to adjust these values)
        chain.addSegment(Segment(Joint(Joint.RotZ), Frame(Vector(0, 0, 0.0892))))
        chain.addSegment(Segment(Joint(Joint.RotY), Frame(Vector(0, 0.1357, 0))))
        chain.addSegment(Segment(Joint(Joint.RotY), Frame(Vector(0, -0.1197, 0.425))))
        chain.addSegment(Segment(Joint(Joint.RotY), Frame(Vector(0, 0, 0.39225))))
        chain.addSegment(Segment(Joint(Joint.RotZ), Frame(Vector(0, 0.1095, 0))))
        chain.addSegment(Segment(Joint(Joint.RotY), Frame(Vector(0, 0.09465, 0))))
        
        return chain
        
    def timer_callback(self):
        # Convert goal_pose to KDL Frame
        x, y, z, roll, pitch, yaw = self.goal_pose
        frame_goal = Frame(Rotation.RPY(roll, pitch, yaw), Vector(x, y, z))
        
        # Solve inverse kinematics
        joint_angles = [0.0] * 6  # Initial guess
        result = self.ik_solver.CartToJnt(joint_angles, frame_goal)
        
        if result >= 0:  # IK solution found
            msg = JointTrajectory()
            msg.joint_names = self.joints
            point = JointTrajectoryPoint()
            point.positions = list(joint_angles)
            point.time_from_start = Duration(sec=2)
            msg.points.append(point)
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Failed to find IK solution")
        
def main(args=None):
    rclpy.init(args=args)
    node = UR5InverseKinematicsController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()