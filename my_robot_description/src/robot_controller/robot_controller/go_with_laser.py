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

        # Συνδρομή στα δεδομένα του LIDAR
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor/out',  
            self.lidar_callback,
            10)
        
        # Δημοσίευση των δεδομένων στο /scan
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

    def lidar_callback(self, msg):
        self.scan_publisher.publish(msg) 


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Διαγραφή του παλιού αρχείου αν υπάρχει
        self.laser_file_path = '/home/hercules/my_robot_description/laser_data/laser_data.txt'
        if os.path.exists(self.laser_file_path):
            os.remove(self.laser_file_path)
            self.get_logger().info(f"Deleted old file: {self.laser_file_path}")
        
        # Άνοιγμα του αρχείου σε λειτουργία append για καταγραφή δεδομένων
        self.laser_file = open(self.laser_file_path, 'a')

        # Δημιουργία συνδρομής στον LIDAR
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/gazebo_ros_ray_sensor/out',  # Ανάλογα με το topic του LIDAR
            self.lidar_callback,
            10)

        # Δημιουργία συνδρομής στην κάμερα (topic: /camera/image_raw ή ανάλογα με το θέμα της κάμερας)
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # Ανάλογα με το topic της κάμερας
            self.camera_callback,
            10)

        # Δημιουργία publisher για εντολές κίνησης
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.safe_distance = 1.0  # Στόχος απόστασης (μέτρα)

        # Χρησιμοποιούμε Timer για καταγραφή κάθε 10 δευτερόλεπτα
        self.timer = self.create_timer(100.0, self.record_laser_data)
        
        # Δημιουργία του αντικειμένου CvBridge για την μετατροπή εικόνας
        self.bridge = CvBridge()

        # Διαγραφή παλιών εικόνων πριν την αποθήκευση νέων
        self.image_folder_path = '/home/hercules/my_robot_description/image_data/'
        self.delete_old_images()

        # Χρονόμετρο για την καταγραφή εικόνας κάθε 10 δευτερόλεπτα
        self.camera_timer = self.create_timer(100.0, self.capture_image)


    def delete_old_images(self):
        # Διαγραφή των παλιών φωτογραφιών στον φάκελο image_data
        if os.path.exists(self.image_folder_path):
            for file_name in os.listdir(self.image_folder_path):
                file_path = os.path.join(self.image_folder_path, file_name)
                if os.path.isfile(file_path) and file_name.endswith(".jpg"):
                    os.remove(file_path)
                    self.get_logger().info(f"Deleted old image: {file_name}")

    def lidar_callback(self, msg):
        # Υπολογισμός της μέσης απόστασης από τα εμπόδια στο σκανάρισμα
        valid_ranges = [distance for distance in msg.ranges if distance > 0]  # Αποφυγή μη έγκυρων δεδομένων
        if len(valid_ranges) > 0:
            average_distance = sum(valid_ranges) / len(valid_ranges)  # Υπολογισμός μέσης απόστασης

            twist = Twist()

            # Καταγραφή των δεδομένων laser στο αρχείο
            laser_data = f"Ranges: {valid_ranges}\n"
            self.laser_file.write(laser_data)

            
            if average_distance < self.safe_distance:
                self.get_logger().info(f"Too close! Distance: {average_distance:.2f}. Turning right...")
                twist.linear.x = 0.2
                twist.angular.z = -0.5  # Στροφή δεξιά

            
            elif average_distance > self.safe_distance:
                self.get_logger().info(f"Too far! Distance: {average_distance:.2f}. Turning left...")
                twist.linear.x = 0.2
                twist.angular.z = 0.5  # Στροφή αριστερά

            # Αλλιώς, συνεχίζει ευθεία
            else:
                self.get_logger().info(f"At the right distance: {average_distance:.2f}. Moving forward...")
                twist.linear.x = 0.2
                twist.angular.z = 0.0  # Κίνηση ευθεία

            # Δημοσίευση της εντολής twist
            self.cmd_publisher.publish(twist)

    def camera_callback(self, msg):
        try:
            # Μετατροπή της εικόνας ROS σε OpenCV χρησιμοποιώντας το CvBridge
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Εάν χρησιμοποιείς "bgr8" για τις έγχρωμες εικόνες
            
            # Δημιουργία ονόματος αρχείου για την αποθήκευση της εικόνας (με timestamp)
            timestamp = time.time()
            image_filename = f"/home/hercules/my_robot_description/image_data/image_{int(timestamp)}.jpg"
            
            # Αποθήκευση της εικόνας στο δίσκο
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info(f"Image saved at: {image_filename}")

        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

    def capture_image(self):
        # Λήψη φωτογραφίας κάθε 10 δευτερόλεπτα
        self.get_logger().info("Capturing photo...")

        # Ενεργοποίηση της συνδρομής για τη λήψη της εικόνας (θα γίνει με την camera_callback)
        self.camera_subscriber

        self.get_logger().info("Photo captured and saved.")

    def record_laser_data(self):
        # Λήψη των τρεχόντων δεδομένων LIDAR κάθε 10 δευτερόλεπτα
        laser_data = "Recorded laser data at timestamp\n"
        self.laser_file.write(laser_data)
        self.get_logger().info("Laser data recorded.")

    def __del__(self):
        # Κλείσιμο του αρχείου όταν καταστραφεί το node
        if self.laser_file:
            self.laser_file.close()

def main(args=None):
    rclpy.init(args=args)

    # Δημιουργία των δύο nodes
    slam_node = SLAMIntegration()
    obstacle_node = ObstacleAvoidance()

    # Χρήση MultiThreadedExecutor για παράλληλη εκτέλεση
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(slam_node)
    executor.add_node(obstacle_node)

    try:
        executor.spin()  # Τρέχει και τα δύο nodes ταυτόχρονα
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        obstacle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


