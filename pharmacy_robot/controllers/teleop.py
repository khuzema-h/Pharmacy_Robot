#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

# Key bindings for teleoperation
key_bindings = {
    'w': (0.1, 0.0),  # Move joint 1 forward
    's': (-0.1, 0.0), # Move joint 1 backward
    'a': (0.0, 0.1),  # Move joint 2 forward
    'd': (0.0, -0.1)  # Move joint 2 backward
}

def get_key():
    """Reads a single keypress from the terminal."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        self.publisher = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

        # Initial joint positions
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # Adjust for your robot's joints
        self.joint_names = [
            'robot_base', 
            'vertical_rail', 
            'prismatic_base', 
            'prismatic_arm', 
            'gripper_base'
        ]  # Replace with your robot's joint names

        self.get_logger().info("Teleoperation node started. Use 'w/s/a/d' to control the robot. Press 'q' to quit.")

    def send_command(self, joint_index, delta):
        """Sends a joint trajectory command with updated positions."""
        self.joint_positions[joint_index] += delta
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start.sec = 1
        trajectory.points.append(point)

        self.publisher.publish(trajectory)
        self.get_logger().info(f"Updated joint positions: {self.joint_positions}")

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    teleop_node = RobotTeleop()

    try:
        while rclpy.ok():
            key = get_key()
            if key == 'q':  # Quit
                break
            elif key in key_bindings:
                joint_index = 0 if key in ['w', 's'] else 1
                delta = key_bindings[key][0] if key in ['w', 's'] else key_bindings[key][1]
                teleop_node.send_command(joint_index, delta)
    except Exception as e:
        teleop_node.get_logger().error(str(e))
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
