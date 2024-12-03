#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge


class Scanner(Node):
    def __init__(self):
        super().__init__("scanner")

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # Subscriptions
        self.subscription_image = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.subscription_depth = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )

        # Internal state
        self.depth_data = None
        self.qr_center = None
        self.qr_code_detector = cv2.QRCodeDetector()  # QR code detector

    def depth_callback(self, msg):
        """Process depth image and update depth data."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_data = np.nan_to_num(
                depth_image, nan=np.inf
            )  # Replace NaNs with infinity

            # Visualize the depth image
            depth_display = cv2.normalize(
                self.depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_BONE)
            cv2.namedWindow("Depth Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                "Depth Camera", 640, 480
            )  # Adjust width and height as needed

            # Overlay distance if marker center is available
            if self.qr_center is not None:
                cx, cy = int(self.qr_center[0]), int(self.qr_center[1])
                cx = np.clip(cx, 0, self.depth_data.shape[1] - 1)
                cy = np.clip(cy, 0, self.depth_data.shape[0] - 1)

                qr_depth = self.depth_data[cy, cx]
                cv2.putText(
                    depth_colormap,
                    f"Depth: {qr_depth:.2f}m",
                    (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    1,
                )

            cv2.imshow("Depth Camera", depth_colormap)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def image_callback(self, msg):
        """Process RGB image and detect QR Code, then warp it into view."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Use QRCodeDetector to detect QR codes
            retval, decoded_info, points, straight_qrcode = self.qr_code_detector.detectAndDecodeMulti(gray)

            # Create a named window with a specific size
            cv2.namedWindow("Camera Feed With QR Codes", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Camera Feed With QR Codes", 640, 480)

            if retval and decoded_info:
                for i, qr_code in enumerate(decoded_info):
                    # Draw the detected QR code border
                    pts = np.int32(points[i]).reshape(-1, 2)
                    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                    # Calculate the center of the QR code
                    qr_center = np.mean(pts, axis=0)
                    self.qr_center = qr_center

                    # Display decoded information and coordinates
                    cv2.putText(
                        frame,
                        f"QR Code: {qr_code}",
                        (int(qr_center[0]), int(qr_center[1] - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"Coords: ({int(qr_center[0])}, {int(qr_center[1])})",
                        (int(qr_center[0]), int(qr_center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

                    # Perspective warping
                    if pts.shape[0] == 4:  # Ensure there are 4 points for the QR code
                        # Define the destination rectangle (top-down view of the QR code)
                        width = 300  # Define the desired width of the warped image
                        height = 300  # Define the desired height of the warped image
                        dst_pts = np.array([
                            [0, 0],  # Top-left corner
                            [width - 1, 0],  # Top-right corner
                            [width - 1, height - 1],  # Bottom-right corner
                            [0, height - 1]  # Bottom-left corner
                        ], dtype="float32")

                        # Get the perspective transform matrix
                        M = cv2.getPerspectiveTransform(pts.astype("float32"), dst_pts)

                        # Warp the QR code into a top-down view
                        warped_qr = cv2.warpPerspective(frame, M, (width, height))

                        # Display the warped QR code in the window
                        cv2.imshow("Warped QR Code", warped_qr)

            cv2.imshow("Camera Feed With QR Codes", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Scanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
