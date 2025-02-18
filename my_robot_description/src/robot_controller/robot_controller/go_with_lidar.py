import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import os
import cv2
from cv_bridge import CvBridge
import time

from .utils import movement 


def main(args=None):
    rclpy.init(args=args)
    node = movement.ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
