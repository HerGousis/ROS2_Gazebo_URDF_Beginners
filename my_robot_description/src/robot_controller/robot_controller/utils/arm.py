import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
 

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        
        self.publisher = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        
        self.timer = self.create_timer(1.0, self.send_command)

        
        self.angle = 0.0

    def send_command(self):
        
        self.angle += math.radians(30)  
        if self.angle > 2 * math.pi:
            self.angle = 0.0  

        
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = "arm_base_link"
        msg.joint_names = ["arm_base_forearm_joint", "forearm_hand_joint"]

        
        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.0]  
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent command: Base Rotation = {math.degrees(self.angle):.1f}°")
