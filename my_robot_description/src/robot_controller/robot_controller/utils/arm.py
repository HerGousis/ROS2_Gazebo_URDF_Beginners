import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
 

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Δημιουργία publisher για joint commands
        self.publisher = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        # Χρονόμετρο που στέλνει εντολές κάθε 1 δευτερόλεπτο
        self.timer = self.create_timer(1.0, self.send_command)

        # Μεταβλητή για την περιστροφή (ξεκινά από 0 μοίρες)
        self.angle = 0.0

    def send_command(self):
        # Περιστροφή 360° σταδιακά
        self.angle += math.radians(30)  # Προσθέτει 30° σε κάθε βήμα
        if self.angle > 2 * math.pi:
            self.angle = 0.0  # Reset αν ξεπεράσει τις 360°

        # Δημιουργία μηνύματος JointTrajectory
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()  # Προσθήκη header
        msg.header.frame_id = "arm_base_link"
        msg.joint_names = ["arm_base_forearm_joint", "forearm_hand_joint"]

        # Ορίζουμε τις θέσεις των αρθρώσεων
        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.0]  # Η δεύτερη άρθρωση μένει στο 0
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent command: Base Rotation = {math.degrees(self.angle):.1f}°")
