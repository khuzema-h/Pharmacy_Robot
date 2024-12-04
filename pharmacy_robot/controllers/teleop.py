#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile
import sys
import termios
import tty

class TeleopRobot(Node):
    def __init__(self):
        super().__init__('teleop_robot')
        
        qos_profile = QoSProfile(depth=10)
        self.joint_command_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', qos_profile)
        
        # Initialize joint positions and limits
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Start all joints at 0
        self.joint_limits = [
            (0, 2.4),       # robot_base
            (-3.14, 1.57),  # vertical_rail
            (0, 0.75),      # prismatic_base
            (0, 0.5),       # prismatic_arm
            (-0.8, 0),      # robotiq_85_right_knuckle_joint
            (0, 0.8)        # robotiq_85_left_knuckle_joint
        ]
        
        self.gripper_open = True  # Gripper state (open/close)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Teleop Robot node has started.")

    def timer_callback(self):
        """
        Publishes the current joint positions periodically.
        """
        try:
            command_msg = Float64MultiArray()
            command_msg.data = self.joint_positions
            self.joint_command_pub.publish(command_msg)
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

    def update_joint(self, joint_index, delta):
        """
        Updates the position of a specific joint, ensuring it stays within limits.
        """
        try:
            new_position = self.joint_positions[joint_index] + delta
            min_limit, max_limit = self.joint_limits[joint_index]
            self.joint_positions[joint_index] = max(min_limit, min(new_position, max_limit))
            self.get_logger().info(f"Updated joint {joint_index}: {self.joint_positions[joint_index]:.2f}")
        except IndexError:
            self.get_logger().error(f"Invalid joint index: {joint_index}")
        except Exception as e:
            self.get_logger().error(f"Error updating joint {joint_index}: {e}")

    def toggle_gripper(self):
        """
        Toggles the gripper state between open and closed.
        """
        if self.gripper_open:
            # Close gripper: Move both fingers to their respective closed positions
            self.joint_positions[4] = -0.8  # robotiq_85_right_knuckle_joint
            self.joint_positions[5] = 0.8   # robotiq_85_left_knuckle_joint
        else:
            # Open gripper: Move both fingers to their respective open positions
            self.joint_positions[4] = 0.0
            self.joint_positions[5] = 0.0

        self.gripper_open = not self.gripper_open
        state = "open" if self.gripper_open else "closed"
        self.get_logger().info(f"Gripper is now {state}")

def get_key():
    """
    Reads a single keypress from the terminal.
    """
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    teleop_robot = TeleopRobot()
    
    print("Teleop Robot Control")
    print("-------------------")
    print("Use the following keys to control the robot:")
    print("q/a: robot_base")
    print("w/s: vertical_rail")
    print("e/d: prismatic_base")
    print("r/f: prismatic_arm")
    print("t: Toggle gripper (open/close)")
    print("Press 'Esc' to quit")
    
    try:
        key_map = {
            'q': (0, 0.01), 'a': (0, -0.01),
            'w': (1, 0.01), 's': (1, -0.01),
            'e': (2, 0.01), 'd': (2, -0.01),
            'r': (3, 0.01), 'f': (3, -0.01),
        }
        
        while True:
            key = get_key()
            if ord(key) == 27:  # Escape key
                print("Exiting Teleop Control...")
                break
            
            if key in key_map:
                joint_index, delta = key_map[key]
                teleop_robot.update_joint(joint_index, delta)
            elif key == 't':  # Toggle gripper
                teleop_robot.toggle_gripper()
            
            rclpy.spin_once(teleop_robot, timeout_sec=0)
    except KeyboardInterrupt:
        print("Teleop Control interrupted.")
    finally:
        teleop_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
