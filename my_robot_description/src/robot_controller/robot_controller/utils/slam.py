import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import yaml
import os
import time

class SLAMIntegration(Node):
    def __init__(self):
        super().__init__('slam_integration')

        # Ορισμός φακέλου αποθήκευσης
        self.save_path = '/home/hercules/data/laser_data/slam'
        os.makedirs(self.save_path, exist_ok=True)

        # Διαγραφή παλιών χαρτών
        for file in os.listdir(self.save_path):
            file_path = os.path.join(self.save_path, file)
            if os.path.isfile(file_path):
                os.remove(file_path)

        # Συνδρομή στα δεδομένα του LIDAR
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor2/out',  # Το θέμα των δεδομένων LIDAR
            self.lidar_callback,
            10)
        
        # Δημοσίευση των δεδομένων LIDAR
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Συνδρομή στα δεδομένα του χάρτη
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',  # Το θέμα του χάρτη
            self.map_callback,
            10)

    def lidar_callback(self, msg):
        """Δημοσιεύει τα δεδομένα LIDAR στο /scan"""
        self.scan_publisher.publish(msg)

    def map_callback(self, msg):
        """Αποθηκεύει τον χάρτη από το OccupancyGrid"""
        self.save_map(msg)

    def save_map(self, msg):
        """Αποθηκεύει τον χάρτη σε μορφή εικόνας (.pgm) και αρχείο YAML"""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        # Μετατροπή δεδομένων σε numpy array
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Αντιστροφή του άξονα Υ (για σωστή προβολή)
        map_data = np.flipud(map_data)

        # Αντικατάσταση τιμών (-1 = άγνωστο, 0 = ελεύθερο, 100 = εμπόδιο)
        image_data = np.zeros((height, width), dtype=np.uint8)
        image_data[map_data == -1] = 205  # Γκρι για άγνωστες περιοχές
        image_data[map_data == 0] = 255   # Λευκό για ελεύθερες περιοχές
        image_data[map_data == 100] = 0   # Μαύρο για εμπόδια

        # Δημιουργία ονόματος αρχείου με timestamp
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        image_filename = f'map_{timestamp}.pgm'
        yaml_filename = f'map_{timestamp}.yaml'

        # Αποθήκευση της εικόνας
        cv2.imwrite(os.path.join(self.save_path, image_filename), image_data)

        # Δημιουργία αρχείου YAML με τις πληροφορίες του χάρτη
        map_metadata = {
            'image': image_filename,
            'resolution': resolution,
            'origin': [origin.x, origin.y, origin.z],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        with open(os.path.join(self.save_path, yaml_filename), 'w') as yaml_file:
            yaml.dump(map_metadata, yaml_file, default_flow_style=False)

        self.get_logger().info(f"Ο χάρτης αποθηκεύτηκε ως {image_filename} και {yaml_filename}")
