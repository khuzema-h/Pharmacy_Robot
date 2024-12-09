#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile
import time


class TeleopRobot(Node):
    def __init__(self):
        super().__init__('teleop_robot')

        qos_profile = QoSProfile(depth=10)
        self.joint_command_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', qos_profile)

        # Initialize joint positions
        self.joint_positions = [0.0] * 6 # Initialize all joints to 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Teleop Robot node has started.")

    def timer_callback(self):
        """
        Publishes the current joint positions periodically.
        """
        try:
            command_msg = Float64MultiArray()
            command_msg.data = [float(pos) for pos in self.joint_positions] # Ensure all are floats
            self.joint_command_pub.publish(command_msg)
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

    def move_to_checkpoint(self, positions, gripper_closed, delay):
        """
        Moves the robot to the specified joint positions and gripper state.

        Args:
            positions (list): Joint positions [joint0, joint1, joint2, joint3, joint4, joint5].
            gripper_closed (bool): True to close the gripper, False to open it.
            delay (float): Time to wait after reaching the position (in seconds).
        """
        try:
            # Validate and assign joint positions
            for i, pos in enumerate(positions):
                self.joint_positions[i] = float(pos) # Explicitly convert each value to float

            # Update gripper state
            if gripper_closed:
                self.joint_positions[4] = -0.1 # robotiq_85_right_knuckle_joint (close)
                self.joint_positions[5] = 0.1 # robotiq_85_left_knuckle_joint (close)
            else:
                self.joint_positions[4] = 0.0 # robotiq_85_right_knuckle_joint (open)
                self.joint_positions[5] = 0.0 # robotiq_85_left_knuckle_joint (open)

            self.get_logger().info(f"Moving to position: {self.joint_positions} | Gripper {'closed' if gripper_closed else 'open'}")
            time.sleep(delay) # Wait for the specified delay
        except ValueError as ve:
            self.get_logger().error(f"Invalid joint position value: {ve}")
        except Exception as e:
            self.get_logger().error(f"Error in move_to_checkpoint: {e}")


def main():
    rclpy.init()
    teleop_robot = TeleopRobot()

    try:
        # Move to the 1st checkpoint
        teleop_robot.move_to_checkpoint(
            positions=[1.35, 0.0, 0.35, 0.08, 0.0, 0.0],
            gripper_closed=True,
            delay=2.0 # Wait for 2 seconds at this position
        )

        # Move to the 2nd checkpoint
        teleop_robot.move_to_checkpoint(
            positions=[2.4, 1.57, 0.0, 0.0, 0.0, 0.0],
            gripper_closed=False,
            delay=2.0 # Wait for 2 seconds at this position
        )

    except KeyboardInterrupt:
        print("Teleop Robot interrupted.")
    finally:
        teleop_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
