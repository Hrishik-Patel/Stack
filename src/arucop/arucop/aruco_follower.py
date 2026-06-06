#!/usr/bin/env python3

import math
import threading
import numpy as np

import cv2
import cv_bridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8


# =========================
# TUNABLE PARAMETERS
# =========================

MAX_LINEAR_VEL = 0.8
MIN_LINEAR_VEL = 0.2

MAX_ANGULAR_VEL = 0.6
MIN_ANGULAR_VEL = 0.2

STOP_DISTANCE = 0.6
FOLLOW_DISTANCE = 4.0


class ArucoFollower(Node):

    def __init__(self):
        super().__init__('aruco_follower')

        self.bridge = cv_bridge.CvBridge()

        self.depth_image = None
        self.latest_ids = []

        self.target_id = None
        self.waiting_for_input = False

        self.rgb_sub = self.create_subscription(
            CompressedImage,
            '/zed/zed_node/rgb/color/rect/image/compressed',
            self.rgb_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.detected_pub = self.create_publisher(
            Int8,
            '/aruco_detected',
            10)

        self.timer = self.create_timer(
            2.0,
            self.timer_callback)

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_5X5_100)

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters)

        self.get_logger().info(
            'Python ArUco Depth Follower Started')

    def depth_callback(self, msg):

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='32FC1')

        except Exception as e:
            self.get_logger().error(
                f'Depth conversion failed: {e}')

    def rgb_callback(self, msg):

        if self.waiting_for_input:
            self.stop_robot()
            return

        try:

            np_arr = np.frombuffer(
                msg.data,
                dtype=np.uint8)

            frame = cv2.imdecode(
                np_arr,
                cv2.IMREAD_COLOR)

            if frame is None:
                return

            corners, ids, _ = self.detector.detectMarkers(
                frame)

        except Exception as e:

            self.get_logger().error(
                f'ArUco detection failed: {e}')
            return

        self.latest_ids = []

        if ids is not None:
            self.latest_ids = [
                int(marker_id[0])
                for marker_id in ids
            ]

        cmd = Twist()

        detected_msg = Int8()
        detected_msg.data = 0

        if self.target_id is not None and ids is not None:

            ids_list = [
                int(marker_id[0])
                for marker_id in ids
            ]

            if self.target_id in ids_list:

                detected_msg.data = 1

                idx = ids_list.index(
                    self.target_id)

                center_x = int(
                    sum(
                        p[0]
                        for p in corners[idx][0]
                    ) / 4.0)

                center_y = int(
                    sum(
                        p[1]
                        for p in corners[idx][0]
                    ) / 4.0)

                if self.depth_image is not None:

                    center_x = max(
                        0,
                        min(
                            center_x,
                            self.depth_image.shape[1] - 1))

                    center_y = max(
                        0,
                        min(
                            center_y,
                            self.depth_image.shape[0] - 1))

                    depth = float(
                        self.depth_image[
                            center_y,
                            center_x])

                    self.get_logger().info(
                        f'ID={self.target_id} '
                        f'Depth={depth:.2f} m')

                    if math.isfinite(depth):

                        if depth <= STOP_DISTANCE:

                            cmd.linear.x = 0.0
                            cmd.angular.z = 0.0

                        elif depth <= FOLLOW_DISTANCE:

                            # linear velocity = distance * 0.5
                            cmd.linear.x = depth * 0.5

                            cmd.linear.x = max(
                                MIN_LINEAR_VEL,
                                min(
                                    MAX_LINEAR_VEL,
                                    cmd.linear.x
                                )
                            )

                            cmd.angular.z = 0.0

                        else:

                            cmd.linear.x = 0.0

                            # angular velocity scales with distance
                            cmd.angular.z = depth * 0.1

                            cmd.angular.z = max(
                                MIN_ANGULAR_VEL,
                                min(
                                    MAX_ANGULAR_VEL,
                                    cmd.angular.z
                                )
                            )

            else:

                self.get_logger().warn(
                    'Target lost')

                self.target_id = None

        self.get_logger().info(
            f'cmd_vel -> linear.x={cmd.linear.x:.3f}, '
            f'angular.z={cmd.angular.z:.3f}'
        )

        self.detected_pub.publish(
            detected_msg)

        self.cmd_pub.publish(
            cmd)

    def timer_callback(self):

        if (
            self.target_id is None
            and len(self.latest_ids) > 0
            and not self.waiting_for_input
        ):

            self.waiting_for_input = True

            def get_input():

                print(
                    f'\nDetected IDs: '
                    f'{self.latest_ids}')

                try:

                    self.target_id = int(
                        input(
                            'Select target ID: '))

                    print(
                        f'Locked on '
                        f'{self.target_id}')

                except Exception:
                    pass

                self.waiting_for_input = False

            threading.Thread(
                target=get_input,
                daemon=True).start()

    def stop_robot(self):

        self.cmd_pub.publish(
            Twist())

    def destroy_node(self):

        self.stop_robot()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = ArucoFollower()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()