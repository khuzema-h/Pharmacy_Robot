#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander.conversions import pose_to_list
from datetime import datetime

class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # MoveIt interfaces
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "manipulator"  # Change to your MoveIt group name
        self.move_group = MoveGroupCommander(self.group_name)

        # Subscriber to get QR code data from scanner
        self.subscription = self.create_subscription(
            String, "/qr_code_data", self.qr_code_callback, 10
        )

        # Internal state
        self.target_pose = None
        self.qr_data = None

    def qr_code_callback(self, msg):
        """Handle QR code data."""
        self.qr_data = msg.data
        self.get_logger().info(f"Received QR Code data: {self.qr_data}")

        # Extract expiration date and validate
        if self.is_valid_tablet(self.qr_data):
            self.get_logger().info("Tablet is valid. Picking and placing in the bin.")
            self.pick_and_place_action()
        else:
            self.get_logger().info("Tablet is expired. Skipping action.")

    def is_valid_tablet(self, qr_data):
        """Check if the tablet has expired."""
        try:
            # Assuming QR data contains expiration date in "YYYY-MM-DD" format
            expiration_date = datetime.strptime(qr_data, "%Y-%m-%d")
            today = datetime.today()
            return expiration_date >= today
        except ValueError:
            self.get_logger().error("Invalid QR data format. Unable to parse date.")
            return False

    def pick_and_place_action(self):
        """Perform pick and place action."""

        # Set the target pose for the picking action
        self.get_logger().info("Moving to pick position...")
        pick_pose = Pose()
        pick_pose.position.x = 0.5  # Set to shelf x-coordinate
        pick_pose.position.y = 0.0  # Set to shelf y-coordinate
        pick_pose.position.z = 0.2  # Set to shelf z-coordinate
        pick_pose.orientation.w = 1.0

        self.move_to_pose(pick_pose)

        # Simulate gripper closing
        self.get_logger().info("Closing gripper...")
        self.close_gripper()

        # Set the target pose for the placing action
        self.get_logger().info("Moving to place position...")
        place_pose = Pose()
        place_pose.position.x = 0.0  # Set to bin x-coordinate
        place_pose.position.y = 0.5  # Set to bin y-coordinate
        place_pose.position.z = 0.2  # Set to bin z-coordinate
        place_pose.orientation.w = 1.0

        self.move_to_pose(place_pose)

        # Simulate gripper opening
        self.get_logger().info("Opening gripper...")
        self.open_gripper()

        # Return to home position
        self.get_logger().info("Returning to home position...")
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)

    def move_to_pose(self, target_pose):
        """Move the manipulator to the target pose."""
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if not success:
            self.get_logger().error("Failed to reach the target pose.")

    def close_gripper(self):
        """Simulate gripper closing."""
        # Replace with actual gripper command
        self.get_logger().info("Gripper closed.")

    def open_gripper(self):
        """Simulate gripper opening."""
        # Replace with actual gripper command
        self.get_logger().info("Gripper opened.")


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
