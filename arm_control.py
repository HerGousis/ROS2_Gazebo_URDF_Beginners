#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import tkinter as tk

class IKSliderArm(Node):
    def __init__(self):
        super().__init__('ik_slider_arm')
        self.pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        # Μήκη τμημάτων
        self.L1 = 0.6
        self.L2 = 0.6

        # === GUI ===
        self.root = tk.Tk()
        self.root.title("3-DOF Arm IK Control")

        tk.Label(self.root, text="Χειρισμός άκρου βραχίονα (x,z,φ)").pack(pady=5)

        self.x_slider = tk.Scale(self.root, from_=0.0, to=1.2, resolution=0.01,
                                 orient="horizontal", label="X (m)", length=300,
                                 command=self.update_position)
        self.x_slider.set(0.4)
        self.x_slider.pack(pady=5)

        self.z_slider = tk.Scale(self.root, from_=0.0, to=1.2, resolution=0.01,
                                 orient="horizontal", label="Z (m)", length=300,
                                 command=self.update_position)
        self.z_slider.set(0.3)
        self.z_slider.pack(pady=5)

        # ΝΕΟ slider για προσανατολισμό καρπού
        self.phi_slider = tk.Scale(self.root, from_=0, to=360, resolution=1,
                                   orient="horizontal", label="Φ (° - προσανατολισμός)", length=300,
                                   command=self.update_position)
        self.phi_slider.set(0)
        self.phi_slider.pack(pady=5)

        self.status = tk.Label(self.root, text="")
        self.status.pack(pady=5)

        self.root.after(100, self.ros_spin)
        self.root.mainloop()

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(50, self.ros_spin)

    def update_position(self, event=None):
        x = self.x_slider.get()
        z = self.z_slider.get()
        phi_deg = self.phi_slider.get()
        phi = math.radians(phi_deg)

        theta1, theta2 = self.inverse_kinematics(x, z)
        if theta1 is None:
            self.status.config(text="❌ Εκτός εμβέλειας!", fg="red")
            return

        # Υπολογισμός γωνίας καρπού
        theta3 = phi - (theta1 + theta2)

        self.status.config(
            text=f"x={x:.2f} m, z={z:.2f} m, φ={phi_deg:.1f}° → θ1={math.degrees(theta1):.1f}°, θ2={math.degrees(theta2):.1f}°, θ3={math.degrees(theta3):.1f}°",
            fg="green"
        )
        self.publish_trajectory(theta1, theta2, theta3)

    def inverse_kinematics(self, x, z):
        L1, L2 = self.L1, self.L2
        dist = math.sqrt(x**2 + z**2)
        if dist > (L1 + L2) or dist < abs(L1 - L2):
            return None, None
        cos_t2 = (x**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_t2 = max(min(cos_t2, 1.0), -1.0)
        t2 = math.acos(cos_t2)
        k1 = L1 + L2 * math.cos(t2)
        k2 = L2 * math.sin(t2)
        t1 = math.atan2(z, x) - math.atan2(k2, k1)
        return t1, t2

    def publish_trajectory(self, t1, t2, t3):
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.frame_id = 'base_footprint'
        traj.joint_names = [
            'arm_base_forearm_joint',
            'forearm_hand_joint',
            'base_camera_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = [t1, t2, t3]
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    IKSliderArm()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
