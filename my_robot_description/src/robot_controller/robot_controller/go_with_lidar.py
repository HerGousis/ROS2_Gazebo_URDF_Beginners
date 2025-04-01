import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import os
import cv2
from cv_bridge import CvBridge
import time

from .utils import movement , slam #, arm



def main(args=None):
    rclpy.init(args=args)

    # Δημιουργία των δύο nodes
    slam_node = slam.SLAMIntegration()
    obstacle_node = movement.ObstacleAvoidance()
    #arm_node = arm.ArmController()

    # Χρήση MultiThreadedExecutor για παράλληλη εκτέλεση
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(slam_node)
    executor.add_node(obstacle_node)
    #executor.add_node(arm_node)

    try:
        executor.spin()  # Τρέχει και τα δύο nodes ταυτόχρονα
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        obstacle_node.destroy_node()
        #arm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# def main(args=None):
#     rclpy.init(args=args)
#     node = movement.ObstacleAvoidance()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
