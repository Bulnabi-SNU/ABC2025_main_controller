#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import required libraries
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import pyzed.sl as sl  # Requires ZED SDK installation
import torch
import yaml
import numpy as np
import math

# Import required ROS messages
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg._point_cloud2 as pc2
from my_msgs.msg import YoloDetection, DepthActivated
from geometry_msgs.msg import Point  # For publishing the target coordinate

class YOLO_depth(Node):

    def __init__(self):
        super().__init__('YOLO_depth_detection')
        
        # Initialize variables
        self.previous_bboxes = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.raw_image = None
        self.depth_image = None

        # Load camera calibration parameters
        path_to_calibration_file = './src/camera_calibration/zed_calibration_formatted.yaml'
        with open(path_to_calibration_file, 'r') as file:
            try:
                calibration_data = yaml.safe_load(file)
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error loading YAML file: {e}")
                calibration_data = None

        if calibration_data is not None:
            self.K = calibration_data['k']
            self.fx = self.K[0][0]
            self.fy = self.K[1][1]
            self.cx = self.K[0][2]
            self.cy = self.K[1][2]
        else:
            self.get_logger().error("Calibration data not loaded properly.")

        # Initialize the YOLO object detection model
        path_to_pt = './src/yolo_detection/yolo_detection/best.pt'
        self.model = YOLO(path_to_pt)

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers for RGB and depth images
        self.image_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/rgb_raw/image_raw_color',
            self.image_callback,
            qos_profile
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            qos_profile
        )

        # Create publishers for detection, depth activation, and target coordinate
        self.yolo_publisher = self.create_publisher(YoloDetection, '/yolo_detection', qos_profile)
        self.depth_activation_publisher = self.create_publisher(DepthActivated, '/depth_activated', qos_profile)
        self.target_publisher = self.create_publisher(Point, '/target', qos_profile)

        # Setup timer to process incoming images
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
            text_x, text_y = x_min, y_min - 10
            text_color = (0, 255, 0)

            cv2.rectangle(self.raw_image, (x_min, y_min), (x_max, y_max), color, thickness)
            cv2.rectangle(self.raw_image, (text_x, text_y - text_size[1]), (text_x + text_size[0], text_y), text_color, -1)
            cv2.putText(self.raw_image, label, (text_x, text_y), font, font_scale, (0, 0, 0), font_thickness)


    def pixel_to_3d(self, pixel):
        # Convert a pixel coordinate to a 3D point using camera intrinsics and the depth value.
        u, v = pixel
        Z = self.depth_image[v, u]
        if Z <= 0 or Z == np.inf or np.isnan(Z):
            self.get_logger().warn(f"Invalid depth value at pixel ({u}, {v})")
            X = float('nan')
            Y = float('nan')
        else:
            X = Z * (u - self.cx) / self.fx
            Y = Z * (v - self.cy) / self.fy
        return (X, Y, Z)


    def image_callback(self, msg):
        try:
            image = CvBridge().imgmsg_to_cv2(msg, "rgb8")
            self.raw_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.height, self.width = self.raw_image.shape[:2]
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")


    def depth_callback(self, msg):
        try:
            self.depth_image = CvBridge().imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")


    def main_timer_callback(self):
        if self.depth_image is not None and self.raw_image is not None:
            results = self.model(self.raw_image)
            original_image = results[0].orig_img
            bboxes = results[0].boxes.data

            # Publish depth activation flag
            depth_activated_msg = DepthActivated()
            depth_activated_msg.depth_activated = (bboxes.size(0) > 0)
            self.depth_activation_publisher.publish(depth_activated_msg)

            if bboxes.size(0) == 0:
                # When no detection, publish a "null" target (using NaN values)
                target_msg = Point()
                target_msg.x = float('nan')
                target_msg.y = float('nan')
                target_msg.z = float('nan')
                self.target_publisher.publish(target_msg)
                self.get_logger().info("No target detected, published null target.")
            else:
                for bbox in bboxes:
                    x_min, y_min, x_max, y_max, confidence, class_index = bbox.tolist()
                    x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)

                    # Calculate center pixel of the bounding box
                    center_x = int((x_min + x_max) / 2)
                    center_y = int((y_min + y_max) / 2)

                    # Convert the center pixel to 3D coordinates
                    center_3d = self.pixel_to_3d((center_x, center_y))
                    # Compute Euclidean distance (optional)
                    distance = math.sqrt(center_3d[0]**2 + center_3d[1]**2 + center_3d[2]**2)

                    class_name = "balloon"
                    self.get_logger().info(f"3D coordinate of {class_name}: {center_3d} (distance: {distance:.2f})")

                    # Publish YoloDetection message
                    yolo_detection_msg = YoloDetection()
                    yolo_detection_msg.label = class_name
                    yolo_detection_msg.screen_width = float(640)
                    yolo_detection_msg.screen_height = float(360)
                    yolo_detection_msg.xmax = float(x_max)
                    yolo_detection_msg.ymax = float(y_max)
                    yolo_detection_msg.xmin = float(x_min)
                    yolo_detection_msg.ymin = float(y_min)
                    self.yolo_publisher.publish(yolo_detection_msg)

                    # Publish the target's 3D coordinate on the /target topic using geometry_msgs/Point
                    target_msg = Point()
                    target_msg.x = center_3d[0]
                    target_msg.y = center_3d[1]
                    target_msg.z = center_3d[2]
                    self.target_publisher.publish(target_msg)

            self.draw_bboxes(bboxes=bboxes)
            cv2.imshow("Detection Results", original_image)
            cv2.waitKey(1)
        else:
            self.get_logger().info("Waiting for both raw and depth images.")


def main(args=None):
    rclpy.init(args=args)
    node = YOLO_depth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

