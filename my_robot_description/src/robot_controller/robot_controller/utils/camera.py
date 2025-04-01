import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import os
import cv2
from cv_bridge import CvBridge
import time

class Camera:
    def __init__(self, node):
        self.node = node  # ROS2 node για logging
        self.bridge = CvBridge()
        self.image_folder_path = "/home/hercules/data/image_data/"

    def delete_old_images(self):
        if os.path.exists(self.image_folder_path):
            for file_name in os.listdir(self.image_folder_path):
                file_path = os.path.join(self.image_folder_path, file_name)
                if os.path.isfile(file_path) and file_name.endswith(".jpg"):
                    os.remove(file_path)
                    self.node.get_logger().info(f"Deleted old image: {file_name}")

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = int(time.time())
            image_filename = f"{self.image_folder_path}/image_{timestamp}.jpg"
            cv2.imwrite(image_filename, cv_image)
            self.node.get_logger().info(f"Image saved at: {image_filename}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to save image: {e}")

    def capture_image(self):
        self.node.get_logger().info("Capturing photo...")
        self.node.get_logger().info("Photo captured and saved.")
