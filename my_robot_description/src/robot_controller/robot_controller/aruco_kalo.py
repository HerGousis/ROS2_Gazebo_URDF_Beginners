import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import numpy as np
from tf2_ros import TransformBroadcaster

class ArucoLidarDetector(Node):
    def __init__(self):
        super().__init__('aruco_lidar_detector')
        self.bridge = CvBridge()

        # Συνδρομές
        self.subscription = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/gazebo_ros_ray_sensor2/out', self.lidar_callback, 10)

        # Εκδόσεις Pose και TF
        self.pose_publisher = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Ορισμός Αρχείου ArUco Dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # Καλιμπραρισμένος πίνακας καμερας (προσαρμόστε τον αν χρειάζεται)
        self.camera_matrix = np.array([[600, 0, 320], 
                                       [0, 600, 240], 
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))

        # Δεδομένα LiDAR
        self.lidar_ranges = []

    def lidar_callback(self, msg):
        # Αποθήκευση των δεδομένων LiDAR (μπορείς να τα χρησιμοποιήσεις αργότερα)
        self.lidar_ranges = msg.ranges

    def image_callback(self, msg):
        # Μετατροπή εικόνας από ROS σε OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Ανίχνευση των ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            marker_size = 0.21  # Μέγεθος του ArUco marker σε μέτρα
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.dist_coeffs)

            # Επεξεργασία κάθε marker
            for i, marker_id in enumerate(ids.flatten()):
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_link_optical"

                # Υπολογισμός θέσης του marker
                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])

                # Μετατροπή του διανύσματος περιστροφής σε quaternion


                
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])

                R_y_180 = np.array([
                  [-1, 0,  0],
                  [ 0, 1,  0],
                  [ 0, 0, -1]
                ])

                R_x90 = np.array([
                  [1, 0,  0],
                  [0, 0, 1],
                  [0, -1,  0]
                 ]) 
                
                R_x_45 = np.array([
                 [1, 0, 0],
                 [0, np.sqrt(2)/2, np.sqrt(2)/2],
                 [0, -np.sqrt(2)/2, np.sqrt(2)/2]
                 ])
                
                R_z90 = np.array([
                  [0, -1, 0],
                  [1, 0, 0],
                  [0, 0, 1]
                 ])
                
                
                R_z_neg45 = np.array([
                 [np.sqrt(2)/2, np.sqrt(2)/2, 0],
                 [-np.sqrt(2)/2, np.sqrt(2)/2, 0],
                 [0, 0, 1]
                 ])
                
                rotated_matrix = R_y_180 @ R_x90 @ R_x_45 @ R_z90 @ R_x_45  @ R_z_neg45 @  rotation_matrix 


                quaternion = self.rotation_matrix_to_quaternion(rotated_matrix)





                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

                # Δημοσίευση Pose στο /aruco_marker_pose
                self.pose_publisher.publish(pose_msg)

                # Δημοσίευση TF
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_link_optical"
                t.child_frame_id = f"aruco_marker_{marker_id}"
                t.transform.translation.x = pose_msg.pose.position.x
                t.transform.translation.y = pose_msg.pose.position.y
                t.transform.translation.z = pose_msg.pose.position.z
                t.transform.rotation = pose_msg.pose.orientation

                # Δημιουργία του μετασχηματισμού (TF) για το marker
                self.tf_broadcaster.sendTransform(t)

                self.get_logger().info(f"Published pose for marker {marker_id}: {pose_msg.pose.position}")

        cv2.imshow('ArUco Detection', frame)
        cv2.waitKey(1)

    def rotation_matrix_to_quaternion(self, R):
        q_w = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        q_x = (R[2, 1] - R[1, 2]) / (4.0 * q_w)
        q_y = (R[0, 2] - R[2, 0]) / (4.0 * q_w)
        q_z = (R[1, 0] - R[0, 1]) / (4.0 * q_w)
        return [q_x, q_y, q_z, q_w]

    def transform_marker_to_base(self, marker_pose):
        # Μετατροπή από κάμερα σε βάση (π.χ. υποθέτουμε μια στατική μετατροπή)
        camera_to_base_transform = np.array([
            [0.707, -0.707, 0, 1.0],  # Μήτρα περιστροφής (παράδειγμα)
            [0.707, 0.707, 0, 0.5],   # Μήτρα περιστροφής (παράδειγμα)
            [0, 0, 1, 0],              # Μήτρα περιστροφής (παράδειγμα)
            [0, 0, 0, 1]               # Ομογενής συντεταγμένη
        ])

        # Εφαρμογή μετατροπής στη θέση του marker
        transformed_pose = np.dot(camera_to_base_transform, np.array([marker_pose.x, marker_pose.y, marker_pose.z, 1]))
        
        # Δημιουργία PoseStamped για την μετατραπείσα θέση
        pose_msg_base = PoseStamped()
        pose_msg_base.pose.position.x = transformed_pose[0]
        pose_msg_base.pose.position.y = transformed_pose[1]
        pose_msg_base.pose.position.z = transformed_pose[2]

        # Χρησιμοποιούμε τον προσανατολισμό από τον marker
        pose_msg_base.pose.orientation = marker_pose.orientation
        pose_msg_base.header.stamp = self.get_clock().now().to_msg()
        pose_msg_base.header.frame_id = "base_link"  # Ή το όνομα της βάσης του ρομπότ
        return pose_msg_base

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLidarDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
