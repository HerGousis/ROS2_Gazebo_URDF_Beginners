import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        
        # Συνδρομή στην εικόνα από την κάμερα
        self.subscription = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10)

        # Δημοσίευση της πόζας του marker
        self.pose_publisher = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)

        # Φόρτωση του ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # **Παράμετροι κάμερας** (ΠΡΟΣΑΡΜΟΣΕ τις στις ρυθμίσεις της κάμερας σου)
        self.camera_matrix = np.array([[600, 0, 320],  # Focal length x, 0, Principal point x
                                       [0, 600, 240],  # 0, Focal length y, Principal point y
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))  # Υποθέτουμε ότι δεν υπάρχει παραμόρφωση

    def image_callback(self, msg):
        # Μετατροπή της εικόνας σε OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Ανίχνευση ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            # Υπολογισμός πόζας marker (θεωρούμε ότι το marker έχει πλάτος 0.05m = 5cm)
            marker_size = 0.05  # 5 εκατοστά
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                # Σχεδίαση άξονα marker
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)


                # Δημιουργία και δημοσίευση μηνύματος PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_link_optical"  # ΠΡΟΣΑΡΜΟΣΕ το frame αν χρειάζεται

                # Θέση marker
                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])

                # Μετατροπή προσανατολισμού σε quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

                self.pose_publisher.publish(pose_msg)
                self.get_logger().info(f"Published pose for marker {marker_id}: {pose_msg.pose.position}")

        cv2.imshow('ArUco Detection', frame)
        cv2.waitKey(1)

    def rotation_matrix_to_quaternion(self, R):
        """Μετατροπή του rotation matrix σε quaternion"""
        q_w = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        q_x = (R[2, 1] - R[1, 2]) / (4.0 * q_w)
        q_y = (R[0, 2] - R[2, 0]) / (4.0 * q_w)
        q_z = (R[1, 0] - R[0, 1]) / (4.0 * q_w)
        return [q_x, q_y, q_z, q_w]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
