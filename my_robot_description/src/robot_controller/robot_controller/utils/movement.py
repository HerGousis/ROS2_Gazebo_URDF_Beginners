import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import os
import cv2
from cv_bridge import CvBridge
import time

from .lidar import Lidar
from .camera import Camera


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Δημιουργία αντικειμένου κάμερας
        self.camera = Camera(self)  

        # Διαγραφή παλιών εικόνων
        self.camera.delete_old_images()

        # Συνδρομή στην κάμερα
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.camera.camera_callback,
            10
        )

        # Χρονόμετρο για λήψη εικόνας κάθε 10 δευτερόλεπτα
        self.camera_timer = self.create_timer(100.0, self.camera.capture_image)

        #gia lidar

        self.lidar = Lidar(self)

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor/out',  # Ανάλογα με το topic του LIDAR
            self.lidar.lidar_callback,
            10)
        
        
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.safe_distance = 1.0  # Στόχος απόστασης (μέτρα)

        # Χρησιμοποιούμε Timer για καταγραφή κάθε 10 δευτερόλεπτα
        self.timer = self.create_timer(100.0, self.lidar.record_laser_data)
        


        

   