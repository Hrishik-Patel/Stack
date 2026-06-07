#!/usr/bin/env python3

import math
import numpy as np
import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')
        
        # Parameters
        self.declare_parameter('target_id', 0)
        self.target_id = self.get_parameter('target_id').get_parameter_value().integer_value
        self.get_logger().info(f'[INIT] Follower Node started. Target ID: {self.target_id}')

        # Velocity Constants
        self.MAX_LIN, self.MIN_LIN = 0.8, 0.2
        self.MAX_ANG, self.MIN_ANG = 0.6, 0.2

        self.bridge = cv_bridge.CvBridge()
        self.depth_image = None

        self.rgb_sub = self.create_subscription(CompressedImage, '/zed/zed_node/rgb/color/rect/image/compressed', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detected_pub = self.create_publisher(Int8, '/aruco_detected', 10)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())

    def depth_callback(self, msg):
        try: 
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e: 
            self.get_logger().error(f'[DEPTH_ERR] Conversion failed: {e}')

    def rgb_callback(self, msg):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if frame is None: return
        height, width = frame.shape[:2]
        center_x_frame = width / 2

        corners, ids, _ = self.detector.detectMarkers(frame)
        
        cmd = Twist()
        detected = Int8()
        detected.data = 0

        if ids is not None and self.target_id in ids:
            detected.data = 1
            idx = np.where(ids == self.target_id)[0][0]
            
            c = corners[idx][0]
            cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))

            # --- ANGULAR CONTROL (P-Controller with Clamping) ---
            error_x = (cx - center_x_frame) / center_x_frame
            
            if abs(error_x) > 0.05: # Deadband
                target_ang = -1.0 * error_x * 0.7
                sign = 1 if target_ang > 0 else -1
                cmd.angular.z = sign * max(self.MIN_ANG, min(self.MAX_ANG, abs(target_ang)))
            
            # --- LINEAR CONTROL (Clamping) ---
            if self.depth_image is not None:
                depth = float(self.depth_image[cy, cx])
                if math.isfinite(depth) and depth > 0.6:
                    target_lin = depth * 0.5
                    cmd.linear.x = max(self.MIN_LIN, min(self.MAX_LIN, target_lin))
                
            self.get_logger().info(f'[TRACKING] Target {self.target_id} | Dist: {depth:.2f}m | Err: {error_x:.2f}')

        # Publish and Log
            self.cmd_pub.publish(cmd)
            self.detected_pub.publish(detected)
            self.get_logger().info(f'[PUBLISHING] cmd_vel -> linear.x: {cmd.linear.x:.2f}, angular.z: {cmd.angular.z:.2f}, detected: {detected.data}')
        self.detected_pub.publish(detected)
        self.get_logger().info(f'detected: {detected.data}')
def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
