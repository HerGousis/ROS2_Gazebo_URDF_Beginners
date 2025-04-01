import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import os

class Lidar:
    def __init__(self, node):
        self.node = node  # Παίρνει το Node από έξω
        self.safe_distance = 2.0  # Απόσταση ασφαλείας

        # Δημιουργία publisher για εντολές κίνησης
        self.cmd_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.laser_file_path = '/home/hercules/data/laser_data/laser_data1.txt'
        if os.path.exists(self.laser_file_path):
            os.remove(self.laser_file_path)
            self.node.get_logger().info(f"Deleted old file: {self.laser_file_path}")
        
        self.laser_file = open(self.laser_file_path, 'a')

    def lidar_callback(self, msg):
        valid_ranges = [distance for distance in msg.ranges if distance > 0]
        if len(valid_ranges) > 0:
            average_distance = sum(valid_ranges) / len(valid_ranges)
            twist = Twist()

            self.laser_file.write(f"Ranges: {valid_ranges}\n")

            if average_distance < self.safe_distance:
                self.node.get_logger().info(f"Too close! Distance: {average_distance:.2f}. Turning right...")
                twist.linear.x = 0.2
                twist.angular.z = -0.5  
            elif average_distance > self.safe_distance:
                self.node.get_logger().info(f"Too far! Distance: {average_distance:.2f}. Turning left...")
                twist.linear.x = 0.2
                twist.angular.z = 0.5  
            else:
                self.node.get_logger().info(f"At the right distance: {average_distance:.2f}. Moving forward...")
                twist.linear.x = 0.2
                twist.angular.z = 0.0  

            self.cmd_publisher.publish(twist)

    def __del__(self):
        if self.laser_file:
            self.laser_file.close()
    
    def record_laser_data(self):
        # Λήψη των τρεχόντων δεδομένων LIDAR κάθε 10 δευτερόλεπτα
        laser_data = "Recorded laser data at timestamp\n"
        self.laser_file.write(laser_data)
        self.node.get_logger().info("Laser data recorded.")
