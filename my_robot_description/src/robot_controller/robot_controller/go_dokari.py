#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import os
import cv2
from cv_bridge import CvBridge
import time
from nav_msgs.msg import OccupancyGrid


class SLAMIntegration(Node):
    def __init__(self):
        super().__init__('slam_integration')

        # Συνδρομή στα δεδομένα του LIDAR
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor/out',
            self.lidar_callback,
            10)

        # Δημοσίευση των δεδομένων στο /scan (π.χ. δεύτερο topic)
        self.scan_publisher = self.create_publisher(LaserScan, '/gazebo_ros_ray_sensor2/out', 10)

    def lidar_callback(self, msg):
        # Αναμετάδοση των δεδομένων LIDAR (προαιρετικά, μπορεί να αφαιρεθεί κι αυτό)
        self.scan_publisher.publish(msg)


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # ----------- Ρύθμιση φακέλου εικόνων -----------
        self.image_folder_path = '/home/hercules/data/image_data/'
        os.makedirs(self.image_folder_path, exist_ok=True)
        self.delete_old_images()

        # ----------- Συνδρομή στο LIDAR -----------
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor/out',
            self.lidar_callback,
            10)

        # ----------- Συνδρομές στις κάμερες -----------
        self.bridge = CvBridge()
        camera_topics = [
            ('/camera_sensor/image_raw', 'image1'),
            ('/camera_sensor3/image_raw', 'image2'),
            ('/camera_sensor4/image_raw', 'image3'),
            ('/camera_sensor5/image_raw', 'image4'),
            ('/camera_sensor6/image_raw', 'image5'),
            ('/camera_sensor7/image_raw', 'image6'),
            ('/camera_sensor8/image_raw', 'image7')
        ]
        self.camera_subscribers = [
            self.create_subscription(Image, topic, lambda msg, p=prefix: self.camera_callback(msg, p), 10)
            for topic, prefix in camera_topics
        ]

        # ----------- Δημοσίευση εντολών κίνησης -----------
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 1.6

        # ----------- Timers για λήψη εικόνων -----------
        self.camera_timer = self.create_timer(100.0, self.capture_image)

    # ==============================================================
    #               Υποστηρικτικές Συναρτήσεις
    # ==============================================================

    def delete_old_images(self):
        if os.path.exists(self.image_folder_path):
            for file_name in os.listdir(self.image_folder_path):
                if file_name.endswith(".jpg"):
                    os.remove(os.path.join(self.image_folder_path, file_name))
                    self.get_logger().info(f"Deleted old image: {file_name}")

    # ==============================================================
    #               Callbacks
    # ==============================================================

    def lidar_callback(self, msg):
        valid_ranges = [d for d in msg.ranges if d > 0]
        if not valid_ranges:
            return

        avg_dist = sum(valid_ranges) / len(valid_ranges)
        twist = Twist()

        if avg_dist < self.safe_distance:
            self.get_logger().info(f"Too close! Distance: {avg_dist:.2f}. Turning right...")
            twist.linear.x = 0.2
            twist.angular.z = -0.25
        elif avg_dist > self.safe_distance:
            self.get_logger().info(f"Too far! Distance: {avg_dist:.2f}. Turning left...")
            twist.linear.x = 0.2
            twist.angular.z = 0.25
        else:
            self.get_logger().info(f"At the right distance: {avg_dist:.2f}. Moving forward...")
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.cmd_publisher.publish(twist)

    def camera_callback(self, msg, prefix):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = int(time.time())
            filename = f"{self.image_folder_path}{prefix}_{timestamp}.jpg"
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved {prefix} image: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save {prefix} image: {e}")

    # ==============================================================
    #               Timed Actions
    # ==============================================================

    def capture_image(self):
        self.get_logger().info("Triggered image capture timer (every 100s).")


def main(args=None):
    rclpy.init(args=args)
    slam_node = SLAMIntegration()
    obstacle_node = ObstacleAvoidance()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(slam_node)
    executor.add_node(obstacle_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        obstacle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
