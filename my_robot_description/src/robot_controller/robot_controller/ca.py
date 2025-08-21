import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class CANavigator(Node):
    def __init__(self):
        super().__init__('ca_navigator')

        # Συνδρομητής LiDAR
        self.subscriber = self.create_subscription(
            LaserScan, '/gazebo_ros_ray_sensor2/out', self.lidar_callback, 10)

        # Δημοσίευση εντολών κίνησης
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Δημοσίευση τοπικού occupancy grid για RViz
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/ca_local_grid', 10)

        # Πλέγμα 30x30 με κελιά 0.2 μέτρα
        self.grid_size = 30
        self.cell_size = 0.2  # μέτρα ανά κελί
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=int)

        # Timer για πλοήγηση
        self.timer = self.create_timer(0.1, self.navigate)

    def lidar_callback(self, msg):
        self.grid.fill(0)
        angle = msg.angle_min

        for r in msg.ranges:
            if 0.1 < r < msg.range_max:
                x = int((r * np.cos(angle)) / self.cell_size + self.grid_size // 2)
                y = int((r * np.sin(angle)) / self.cell_size + self.grid_size // 2)
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    self.grid[x, y] = 1  # εμπόδιο
            angle += msg.angle_increment

    def navigate(self):
        center = self.grid_size // 2
        search_range = 5  # πόσα κελιά μπροστά κοιτάμε
        danger_threshold = 3  # πόσα εμπόδια θεωρείται "κίνδυνος"

        # εμπόδια στην ευθεία μπροστά
        forward_obstacles = np.sum(self.grid[center + 1:center + search_range, center - 2:center + 3])

        twist = Twist()

        if forward_obstacles > danger_threshold:
            # μέτρηση εμποδίων δεξιά και αριστερά
            left = np.sum(self.grid[center - 2:center + 3, center + 2:center + 6])
            right = np.sum(self.grid[center - 2:center + 3, center - 6:center - 2])

            if left < right:
                twist.angular.z = 0.5  # στρίψε αριστερά
            else:
                twist.angular.z = -0.5  # στρίψε δεξιά
        else:
            twist.linear.x = 0.2  # προχώρα ευθεία

        self.cmd_pub.publish(twist)
        self.publish_occupancy_grid()

    def publish_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # βεβαιώσου ότι RViz έχει ίδιο fixed frame

        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size

        offset = (self.grid_size * self.cell_size) / 2.0
        msg.info.origin.position.x = -offset
        msg.info.origin.position.y = -offset
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flip & transpose για να έχει σωστό προσανατολισμό
        flipped = np.flipud(self.grid.T)
        msg.data = [100 if val == 1 else 0 for val in flipped.flatten()]

        self.grid_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = CANavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
