# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import sensor_msgs
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

# import other libraries
import numpy as np
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError


class depth_image(Node):

    def __init__(self):
        super().__init__('depth_image_node')

        # Initialize variables
        self.bridge = CvBridge()
        self.depth_image = None
        self.selected_point = None

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers
        self.depth_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10)
        

        cv2.namedWindow("Depth Image")
        cv2.setMouseCallback("Depth Image", self.select_point)
        
        # Timer setup
        self.main_timer = self.create_timer(0.1, self.main_timer_callback)

    # Services
    def select_point(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_point = (x, y)
            self.get_logger().info(f"Point selected: ({x}, {y})")


    # Callback functions for timers


    # Callback functions for subscribers
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
작성된 코드에서 몇 가지 문제가 있습니다. 이를 분석하고 수정해야 프로그램이 제대로 동작합니다.
문제점 분석

    select_point 함수의 호출 오류:
        select_point 함수는 이벤트 콜백으로 설계되었지만, main_timer_callback에서 호출 방식이 잘못되었습니다.
        select_point의 결과를 바로 사용할 수 없습니다.

    bridge 초기화 누락:
        depth_callback에서 self.bridge.imgmsg_to_cv2를 호출하지만, CvBridge 객체 self.bridge를 초기화하지 않았습니다.

    depth_image 변수 초기화 문제:
        depth_image는 depth_callback에서만 할당되지만, 할당 이전에 접근할 경우 문제가 발생합니다.

    ROI (Region of Interest) 범위 정의 문제:
        y_min, y_max, x_min, x_max 등의 변수가 정의되지 않았습니다.

    OpenCV 창과 ROS2의 연동 문제:
        OpenCV 창에서 마우스 클릭 이벤트를 처리하려면 OpenCV의 setMouseCallback을 사용해야 합니다.

수정된 코드

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class DepthImageNode(Node):

    def __init__(self):

        super().__init__('depth_image_node')

        # Initialize Variables
        self.bridge = CvBridge()
        self.depth_image = None
        self.selected_point = None

        # Set QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subcribers
        self.depth_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            qos_profile
        )

        cv2.namedWindow("Depth Image")
        cv2.setMouseCallback("Depth Image", self.select_point)

        self.main_timer = self.create_timer(0.1, self.main_timer_callback)


    def select_point(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_point = (x, y)
            self.get_logger().info(f"Point selected: ({x}, {y})")


    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")


    def main_timer_callback(self):
        if self.depth_image is not None:
            cv2.imshow("Depth Image", self.depth_image)

            if self.selected_point is not None:
                x, y = self.selected_point
                if 0 <= y < self.depth_image.shape[0] and 0 <= x < self.depth_image.shape[1]:
                    depth_value = self.depth_image[y, x]
                    self.get_logger().info(f"Depth value at ({x}, {y}): {depth_value:.2f} meters")
                else:
                    self.get_logger().warning("Selected point is out of bounds.")

            # OpenCV 창 업데이트
            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    depth_image = depth_image()

    rclpy.spin(depth_image)

    depth_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
