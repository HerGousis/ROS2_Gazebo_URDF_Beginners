#!/usr/bin/env python3
import time, random, math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from .utils import movement , slam 
# ------------------ Helpers (δικό σου ελαφρύ RANSAC floor) ------------------
def fit_plane_from_3pts(p1, p2, p3):
    v1, v2 = p2 - p1, p3 - p1
    n = np.cross(v1, v2)
    nrm = np.linalg.norm(n)
    if nrm < 1e-9:
        return None
    n = n / nrm
    d = -float(n @ p1)
    return n, d

def ransac_floor(xyz, distance_thresh=0.02, iters=200, seed=None):
    """RANSAC για επίπεδο δαπέδου (ελαφρύ)."""
    if seed is not None:
        random.seed(seed); np.random.seed(seed)
    N = int(xyz.shape[0])
    if N < 500:
        return None

    best_inliers, best_cnt = None, 0
    best_n, best_d = None, None

    for _ in range(iters):
        i1, i2, i3 = np.random.choice(N, 3, replace=False)
        res = fit_plane_from_3pts(xyz[i1], xyz[i2], xyz[i3])
        if res is None:
            continue
        n, d = res
        dist = np.abs(xyz @ n + d)
        inliers = dist < distance_thresh
        cnt = int(inliers.sum())
        if cnt > best_cnt:
            best_cnt, best_inliers, best_n, best_d = cnt, inliers, n, d

    if best_inliers is None:
        return None

    P = xyz[best_inliers]
    c = P.mean(axis=0, keepdims=True)
    Q = P - c
    _, _, vh = np.linalg.svd(Q, full_matrices=False)
    n = vh[-1, :]
    n = n / (np.linalg.norm(n) + 1e-9)
    d = -float(n @ c.ravel())
    return n, d

# ------------------ Node ------------------
class ArucoFollowLightRansac(Node):
    """
    Ελαφριά έκδοση:
      - Green=floor (RANSAC ανά 10 καρέ)
      - Red=near (<= wall_z_threshold) εμπόδια
      - Blue=far (> wall_z_threshold)
      - ArUco id=target_marker_id -> lock στόχου (κρατάει το κόκκινο ακόμη κι αν χαθεί το marker)
      - P-controller για στοίχιση/προσέγγιση, STOP στα stop_distance_m
    """
    def __init__(self):
        super().__init__('aruco_follow_light_ransac')
        self.bridge = CvBridge()

        # ---------- Parameters ----------
        # Topics
        self.declare_parameter('rgb_topic',   '/camera_sensor2/image_raw')
        self.declare_parameter('depth_topic', '/camera_sensor2/depth/image_raw')
        self.declare_parameter('info_topic',  '/camera_sensor2/depth/camera_info')
        self.declare_parameter('cmd_topic',   '/cmd_vel')

        # ArUco / follow
        self.declare_parameter('target_marker_id', 18)
        self.declare_parameter('stop_distance_m',  1.5)

        # Controller
        self.declare_parameter('kp_ang', 1.0)        # gain στρέψης
        self.declare_parameter('kp_lin', 0.1)        # gain προώθησης
        self.declare_parameter('max_ang', 0.01)       # rad/s
        self.declare_parameter('max_lin', 0.1)       # m/s
        self.declare_parameter('min_lin', 0.06)      # m/s
        self.declare_parameter('heading_gate', 1.0)  # μείωση fwd όταν είμαστε στραβοί

        # Αναζήτηση όταν δεν υπάρχει lock
        self.declare_parameter('search_spin', 0.0)   # rad/s (0 = off)
        self.declare_parameter('lost_frames_stop', 10)

        # Visualization
        self.declare_parameter('viz_near', 0.2)
        self.declare_parameter('viz_far',  4.0)

        # RANSAC/segmentation (όπως στο δικό σου)
        self.declare_parameter('subsample_fit', 8)
        self.declare_parameter('max_fit_points', 30000)
        self.declare_parameter('floor_dist', 0.02)
        self.declare_parameter('ransac_iters', 200)

        self.declare_parameter('min_z', 0.15)
        self.declare_parameter('wall_z_threshold', 5.5)
        self.declare_parameter('min_obj_h', 0.01)
        self.declare_parameter('max_obj_h', 1.50)

        # ---------- Read params ----------
        rgb_topic   = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic  = self.get_parameter('info_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.target_marker_id = int(self.get_parameter('target_marker_id').value)
        self.stop_distance_m  = float(self.get_parameter('stop_distance_m').value)

        self.kp_ang  = float(self.get_parameter('kp_ang').value)
        self.kp_lin  = float(self.get_parameter('kp_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.min_lin = float(self.get_parameter('min_lin').value)
        self.heading_gate = float(self.get_parameter('heading_gate').value)

        self.search_spin = float(self.get_parameter('search_spin').value)
        self.lost_frames_stop = int(self.get_parameter('lost_frames_stop').value)

        self.near_viz = float(self.get_parameter('viz_near').value)
        self.far_viz  = float(self.get_parameter('viz_far').value)

        self.subsample_fit = int(self.get_parameter('subsample_fit').value)
        self.max_fit_points = int(self.get_parameter('max_fit_points').value)
        self.floor_dist = float(self.get_parameter('floor_dist').value)
        self.ransac_iters = int(self.get_parameter('ransac_iters').value)

        self.min_z = float(self.get_parameter('min_z').value)
        self.wall_z_threshold = float(self.get_parameter('wall_z_threshold').value)
        self.min_obj_h = float(self.get_parameter('min_obj_h').value)
        self.max_obj_h = float(self.get_parameter('max_obj_h').value)

        # ---------- QoS ----------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # ---------- Subs/Pubs ----------
        self.sub_info  = self.create_subscription(CameraInfo, info_topic,  self.cb_info,  qos)
        self.sub_depth = self.create_subscription(Image,      depth_topic, self.cb_depth, qos)
        self.sub_rgb   = self.create_subscription(Image,      rgb_topic,   self.cb_rgb,   qos)
        self.cmd_pub   = self.create_publisher(Twist, self.cmd_topic, 10)

        # ---------- Intrinsics / caches ----------
        self.fx = self.fy = self.cx = self.cy = None
        self.W = self.H = None
        self.U = self.V = None
        self.last_rgb_bgr = None

        # ---------- State ----------
        self.frame_count = 0
        self.floor_model = None
        self.alpha = 0.6

        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()  # παλιό API
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()         # νέο API

        # Target lock
        self.target_locked = False
        self.target_mask_locked = None
        self.last_distance_m = math.inf
        self.sent_stop = False
        self.frames_without_depth = 0

        # EMA για σταθερό κέντρο
        self.ema_cx = None
        self.ema_cy = None
        self.ema_beta = 0.6

        # Morph
        self.kernel = np.ones((3,3), np.uint8)

        cv2.namedWindow('Light RANSAC + ArUco Follow (G=floor, R=near, B=far)', cv2.WINDOW_NORMAL)
        self.get_logger().info('Started. Keys: q=quit, s=save.')

    # ---------- Utils ----------
    def publish_cmd(self, lin_x: float, ang_z: float):
        tw = Twist()
        tw.linear.x = float(lin_x)
        tw.angular.z = float(ang_z)
        self.cmd_pub.publish(tw)

    def stop_motion(self, reason: str = ""):
        self.publish_cmd(0.0, 0.0)
        if reason:
            self.get_logger().info(f"STOP: {reason}")

    # ---------- Callbacks ----------
    def cb_info(self, msg: CameraInfo):
        K = msg.k
        self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]
        if (self.W != msg.width) or (self.H != msg.height):
            self.W, self.H = msg.width, msg.height
            u = np.arange(self.W, dtype=np.float32)
            v = np.arange(self.H, dtype=np.float32)
            self.U, self.V = np.meshgrid(u, v)

    def cb_rgb(self, msg: Image):
        try:
            self.last_rgb_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            self.last_rgb_bgr = None

    def cb_depth(self, msg: Image):
        if self.fx is None or self.U is None:
            return

        # --- Depth -> meters ---
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        if msg.encoding in ('16UC1', 'mono16'):
            depth_m = d.astype(np.float32) / 1000.0
        else:
            depth_m = d.astype(np.float32)

        depth_m[~np.isfinite(depth_m) | (depth_m <= 0.0)] = np.nan
        valid = np.isfinite(depth_m)
        if not np.any(valid):
            self.stop_motion("no valid depth")
            return

        H, W = depth_m.shape

        # --- RANSAC floor κάθε 10 frames (ελαφρύ) ---
        self.frame_count += 1
        if self.frame_count % 10 == 0:
            s = self.subsample_fit
            uu_s, vv_s = self.U[::s, ::s], self.V[::s, ::s]
            z_s = depth_m[::s, ::s]
            val_s = np.isfinite(z_s)
            if np.any(val_s):
                zv = z_s[val_s]
                uvs = uu_s[val_s]; vvs = vv_s[val_s]
                xs = (uvs - self.cx) / self.fx * zv
                ys = (vvs - self.cy) / self.fy * zv
                xyz_s = np.stack((xs, ys, zv), axis=1)
                if xyz_s.shape[0] > self.max_fit_points:
                    sel = np.random.choice(xyz_s.shape[0], self.max_fit_points, replace=False)
                    xyz_s = xyz_s[sel]
                self.floor_model = ransac_floor(xyz_s,
                                                distance_thresh=self.floor_dist,
                                                iters=self.ransac_iters)

        # --- Classification (floor / near / far) ---
        floor_mask = np.zeros_like(valid, dtype=bool)
        near_mask  = np.zeros_like(valid, dtype=bool)
        far_mask   = np.zeros_like(valid, dtype=bool)

        height_map = np.full(depth_m.shape, np.nan, dtype=np.float32)

        if self.floor_model is not None:
            n, d0 = self.floor_model
            z = depth_m[valid]
            u = self.U[valid]; v = self.V[valid]
            x = (u - self.cx) / self.fx * z
            y = (v - self.cy) / self.fy * z
            dist_floor = np.abs(x*n[0] + y*n[1] + z*n[2] + d0)
            fm = dist_floor < self.floor_dist
            floor_mask[valid] = fm

            height_signed = (x*n[0] + y*n[1] + z*n[2] + d0)
            height_map[valid] = height_signed

        zfull = np.zeros_like(depth_m, dtype=np.float32)
        zfull[valid] = depth_m[valid]

        # near = valid & not floor & z in [min_z, wall_z_threshold] & ύψος αντικειμένου
        near_mask = valid & (~floor_mask) & (zfull >= self.min_z) & (zfull <= self.wall_z_threshold)
        if self.floor_model is not None:
            near_mask &= (np.abs(height_map) >= self.min_obj_h) & (np.abs(height_map) <= self.max_obj_h)

        # far = valid & not floor & z > wall_z_threshold
        far_mask = valid & (~floor_mask) & (zfull > self.wall_z_threshold)

        # --- ArUco detection & target lock (πάνω στη near_mask) ---
        if self.last_rgb_bgr is not None and self.last_rgb_bgr.shape[:2] == (H, W):
            gray = cv2.cvtColor(self.last_rgb_bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None:
                ids_flat = ids.flatten().tolist()
                cv2.aruco.drawDetectedMarkers(gray, corners, ids)  # μόνο για οπτικοποίηση

                if self.target_marker_id in ids_flat:
                    idx = ids_flat.index(self.target_marker_id)
                    pts = corners[idx].astype(np.int32).reshape(-1, 2)
                    roi_mask = np.zeros((H, W), dtype=np.uint8)
                    cv2.fillConvexPoly(roi_mask, pts, 255)
                    roi_mask = cv2.dilate(roi_mask, np.ones((7,7), np.uint8), iterations=1)
                    roi_bool = roi_mask.astype(bool)

                    # target = near μέσα/κοντά στο ROI
                    target_mask = near_mask & roi_bool
                    if target_mask.sum() < 50:
                        roi_buf = cv2.dilate(roi_mask, np.ones((15,15), np.uint8), iterations=1).astype(bool)
                        target_mask = near_mask & roi_buf

                    if target_mask.any():
                        self.target_locked = True
                        tu8 = (target_mask.astype(np.uint8) * 255)
                        tu8 = cv2.morphologyEx(tu8, cv2.MORPH_CLOSE, self.kernel, iterations=2)
                        self.target_mask_locked = tu8.astype(bool)
                        self.get_logger().info(f"Target LOCKED from ArUco id={self.target_marker_id}")

        # --- Συντήρηση locked στόχου + έλεγχος κίνησης ---
        if self.target_locked and self.target_mask_locked is not None:
            # “κουμπώνει” στο τρέχον near_mask για καθάρισμα
            overlap = self.target_mask_locked & near_mask
            if overlap.sum() > 30:
                self.target_mask_locked = (overlap | self.target_mask_locked)

            # Απόσταση = median βάθος
            t_depths = depth_m[self.target_mask_locked]
            t_depths = t_depths[np.isfinite(t_depths)]
            if t_depths.size > 0:
                self.last_distance_m = float(np.median(t_depths))
                self.frames_without_depth = 0
            else:
                self.frames_without_depth += 1
                self.last_distance_m = math.inf
                if self.frames_without_depth >= self.lost_frames_stop:
                    self.stop_motion("lost depth on target")
                    self.sent_stop = False  # επιτρέπει νέο stop αργότερα

            # Κέντρο μάζας (EMA)
            ys, xs = np.where(self.target_mask_locked)
            if xs.size > 0:
                cx = xs.mean(); cy = ys.mean()
                if self.ema_cx is None:
                    self.ema_cx, self.ema_cy = cx, cy
                else:
                    b = self.ema_beta
                    self.ema_cx = b*self.ema_cx + (1-b)*cx
                    self.ema_cy = b*self.ema_cy + (1-b)*cy

                # Error οριζόντιας στοίχισης [-1..1]
                err_x = ((self.ema_cx) - (W/2)) / max(1.0, (W/2))

                if self.last_distance_m <= self.stop_distance_m:
                    if not self.sent_stop:
                        self.stop_motion(f"target at {self.last_distance_m:.2f} m (<= {self.stop_distance_m:.2f} m)")
                        self.sent_stop = True
                else:
                    # angular
                    ang_cmd = - self.kp_ang * float(err_x)
                    ang_cmd = float(np.clip(ang_cmd, -self.max_ang, self.max_ang))
                    # linear (ανάλογο με dist - stop_d), μειώνεται όταν είμαστε στραβοί
                    fwd = self.kp_lin * float(self.last_distance_m - self.stop_distance_m)
                    fwd = float(np.clip(fwd, 0.0, self.max_lin))
                    fwd *= max(0.0, 1.0 - min(1.0, abs(err_x)/max(1e-6, self.heading_gate)))
                    if fwd > 0.0:
                        fwd = max(fwd, self.min_lin)
                    self.publish_cmd(fwd, ang_cmd)
                    self.sent_stop = False
            else:
                # δεν έχουμε κέντρο μάσκας -> ασφάλεια/αναζήτηση
                if self.search_spin != 0.0 and not self.sent_stop and self.last_distance_m > self.stop_distance_m:
                    self.publish_cmd(0.0, np.sign(self.search_spin)*min(abs(self.search_spin), self.max_ang))
                else:
                    self.stop_motion("no centroid on locked mask")

        else:
            # χωρίς lock: προαιρετικό spin αναζήτησης
            if self.search_spin != 0.0 and not self.sent_stop:
                self.publish_cmd(0.0, np.sign(self.search_spin)*min(abs(self.search_spin), self.max_ang))
            else:
                self.publish_cmd(0.0, 0.0)

        # --- Overlay / Visualization ---
        out = self._make_overlay(depth_m, floor_mask, near_mask, far_mask)
        # ζωγράφισε κέντρο & στόχο
        if self.target_mask_locked is not None:
            out[self.target_mask_locked] = (0, 0, 255)  # έντονο κόκκινο
        if self.target_mask_locked is not None and self.ema_cx is not None and self.ema_cy is not None:
            cv2.circle(out, (int(self.ema_cx), int(self.ema_cy)), 6, (0,0,255), 2)
            cv2.circle(out, (W//2, H//2), 6, (255,255,255), 1)

        status = f'G=floor  R=near  B=far | Target: {"LOCKED" if self.target_locked else "—"}'
        if np.isfinite(self.last_distance_m):
            status += f' | dist={self.last_distance_m:.2f} m (stop@{self.stop_distance_m:.1f}m)'
        cv2.putText(out, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (30,230,30), 2)

        cv2.imshow('Light RANSAC + ArUco Follow (G=floor, R=near, B=far)', out)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            self.stop_motion("manual quit")
            rclpy.shutdown(); cv2.destroyAllWindows()
        elif k == ord('s'):
            ts = int(time.time()); cv2.imwrite(f'light_ransac_follow_{ts}.png', out)
            self.get_logger().info('Saved overlay PNG.')

    # ---------- Viz helper ----------
    def _make_overlay(self, depth_m, floor_mask, near_mask, far_mask):
        H, W = depth_m.shape
        if self.last_rgb_bgr is not None and self.last_rgb_bgr.shape[:2] == (H, W):
            base = self.last_rgb_bgr.copy()
        else:
            gray = np.nan_to_num(depth_m, nan=0.0)
            gray = np.clip((gray - self.near_viz) / (self.far_viz - self.near_viz + 1e-9), 0, 1)
            base = cv2.cvtColor((255*(1-gray)).astype(np.uint8), cv2.COLOR_GRAY2BGR)

        overlay = base.copy()
        overlay[floor_mask] = (0, 255, 0)         # floor = green
        overlay[near_mask]  = (0, 0, 255)         # near = red
        overlay[far_mask]   = (255, 0, 0)         # far  = blue
        out = cv2.addWeighted(overlay, self.alpha, base, 1.0 - self.alpha, 0.0)
        return out

# ------------------ main ------------------
def main():
    rclpy.init()
    node = ArucoFollowLightRansac()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_motion("shutdown")
        except Exception:
            pass
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
