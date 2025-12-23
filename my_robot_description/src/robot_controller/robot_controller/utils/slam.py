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

        
        self.save_path = '/home/hercules/data/laser_data/slam'
        os.makedirs(self.save_path, exist_ok=True)

        
        for file in os.listdir(self.save_path):
            file_path = os.path.join(self.save_path, file)
            if os.path.isfile(file_path):
                os.remove(file_path)

        
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor2/out',  
            self.lidar_callback,
            10)
        
        
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',  
            self.map_callback,
            10)

    def lidar_callback(self, msg):
        
        self.scan_publisher.publish(msg)

    def map_callback(self, msg):
        
        self.save_map(msg)

    def save_map(self, msg):
        
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        
        map_data = np.flipud(map_data)

        
        image_data = np.zeros((height, width), dtype=np.uint8)
        image_data[map_data == -1] = 205  
        image_data[map_data == 0] = 255   
        image_data[map_data == 100] = 0  

        
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        image_filename = f'map_{timestamp}.pgm'
        yaml_filename = f'map_{timestamp}.yaml'

        
        cv2.imwrite(os.path.join(self.save_path, image_filename), image_data)

        
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

        self.get_logger().info(f"{image_filename} & {yaml_filename}")
