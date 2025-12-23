import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import os
import cv2
from cv_bridge import CvBridge
import time



from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class SLAMIntegration(Node):
    def __init__(self):
        super().__init__('slam_integration')

        
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor/out',  
            self.lidar_callback,
            10)
        
        
        self.scan_publisher = self.create_publisher(LaserScan, '/gazebo_ros_ray_sensor2/out', 10)

    def lidar_callback(self, msg):
        self.scan_publisher.publish(msg) 


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        
        self.laser_file_path = '/home/hercules/my_robot_description/laser_data/laser_data.txt'
        if os.path.exists(self.laser_file_path):
            os.remove(self.laser_file_path)
            self.get_logger().info(f"Deleted old file: {self.laser_file_path}")
        
        
        self.laser_file = open(self.laser_file_path, 'a')

        
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor/out',  
            self.lidar_callback,
            10)

        
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  
            self.camera_callback,
            10)

        
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.safe_distance = 1.0  

        self.timer = self.create_timer(100.0, self.record_laser_data)
                
        self.bridge = CvBridge()

        self.image_folder_path = '/home/hercules/my_robot_description/image_data/'
        self.delete_old_images()

        self.camera_timer = self.create_timer(100.0, self.capture_image)

    def delete_old_images(self):
        
        if os.path.exists(self.image_folder_path):
            for file_name in os.listdir(self.image_folder_path):
                file_path = os.path.join(self.image_folder_path, file_name)
                if os.path.isfile(file_path) and file_name.endswith(".jpg"):
                    os.remove(file_path)
                    self.get_logger().info(f"Deleted old image: {file_name}")

    def lidar_callback(self, msg):
        
        valid_ranges = [distance for distance in msg.ranges if distance > 0] 
        if len(valid_ranges) > 0:
            average_distance = sum(valid_ranges) / len(valid_ranges)  

            twist = Twist()

            
            laser_data = f"Ranges: {valid_ranges}\n"
            self.laser_file.write(laser_data)

            
            if average_distance < self.safe_distance:
                self.get_logger().info(f"Too close! Distance: {average_distance:.2f}. Turning right...")
                twist.linear.x = 0.2
                twist.angular.z = -0.5  

            
            elif average_distance > self.safe_distance:
                self.get_logger().info(f"Too far! Distance: {average_distance:.2f}. Turning left...")
                twist.linear.x = 0.2
                twist.angular.z = 0.5  

            
            else:
                self.get_logger().info(f"At the right distance: {average_distance:.2f}. Moving forward...")
                twist.linear.x = 0.2
                twist.angular.z = 0.0  

            
            self.cmd_publisher.publish(twist)

    def camera_callback(self, msg):
        try:
        
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            
            
            timestamp = time.time()
            image_filename = f"/home/hercules/my_robot_description/image_data/image_{int(timestamp)}.jpg"
            
            
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info(f"Image saved at: {image_filename}")

        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

    def capture_image(self):
        
        self.get_logger().info("Capturing photo...")

        self.camera_subscriber

        self.get_logger().info("Photo captured and saved.")

    def record_laser_data(self):
        laser_data = "Recorded laser data at timestamp\n"
        self.laser_file.write(laser_data)
        self.get_logger().info("Laser data recorded.")

    def __del__(self):
        
        if self.laser_file:
            self.laser_file.close()

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


