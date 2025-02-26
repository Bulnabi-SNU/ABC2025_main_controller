#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import required modules
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import yaml

# Import required ROS messages
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


class YOLO_RGB(Node):

    def __init__(self):
        super().__init__('YOLO_rgb_detection')

        # Declare parameters (update the path to your trained weights)
        path_to_pt = './src/yolo_detection/yolo_detection/best.pt'
        self.model = YOLO(path_to_pt)
        
        # Initialize variables
        self.previous_bboxes = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.raw_image = None

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriber for raw RGB image
        self.image_subscriber = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Setup timer to process images periodically
        self.main_timer = self.create_timer(0.2, self.main_timer_callback)


    def draw_bboxes(self, bboxes):
        color = (0, 255, 0)
        thickness = 3

        for row in bboxes:
            x_min, y_min, x_max, y_max, confidence, class_index = row.tolist()
            x_min, y_min, x_max, y_max = map(int, [x_min, y_min, x_max, y_max])
            label = 'balloon'
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1
            text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]
            text_x, text_y = x_min, y_min - 10  # Position label above the bounding box
            text_color = (0, 255, 0)

            cv2.rectangle(self.raw_image, (x_min, y_min), (x_max, y_max), color, thickness)
            cv2.rectangle(self.raw_image, (text_x, text_y - text_size[1]), (text_x + text_size[0], text_y), text_color, -1)
            cv2.putText(self.raw_image, label, (text_x, text_y), font, font_scale, (0, 0, 0), font_thickness)


    def image_callback(self, msg):
        try:
            image = CvBridge().imgmsg_to_cv2(msg, "rgb8")
            # Convert from RGB to BGR for OpenCV processing
            self.raw_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            self.get_logger().error(f"{e}")


    def main_timer_callback(self):
        if self.raw_image is not None:
            results = self.model(self.raw_image)
            bboxes = results[0].boxes.data
            self.get_logger().info(f"bboxes: {bboxes}")
            self.draw_bboxes(bboxes=bboxes)
            cv2.imshow("Object Detection", self.raw_image)
            cv2.waitKey(1)
        else:
            self.get_logger().info("Waiting for image...")


def main(args=None):
    rclpy.init(args=args)
    node = YOLO_RGB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

