# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import required libraries
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import pyzed.sl as sl # Can be imported only if ZED SDK is installed (unavailable in local)
import torch
import yaml
import numpy as np

# import required msgs
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg._point_cloud2 as pc2
from my_msgs.msg import YoloDetection, DepthActivated


class YOLO_depth(Node):

    def __init__(self):
        super().__init__('YOLO_depth_detection')
        
        # Initialize Variables
        self.previous_bboxes = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.raw_image = None
        self.depth_image = None

        # Initialize and open ZED camera
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE 

        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Failed to initialize ZED camera")
            exit()

        # Initialize ZED camera info
        camera_info = self.zed.get_camera_information()
        calibration_params = camera_info.calibration_parameters

        # intrinsic mtx of left camera
        # for pixel_to_3d function
        left_cam_matrix = calibration_params.left_cam
        self.fx = left_cam_matrix.fx
        self.fy = left_cam_matrix.fy
        self.cx = left_cam_matrix.cx
        self.cy = left_cam_matrix.cy

        self.K = [[self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0,  0,  1]]

        # Initialize object detection model
        '''
        rf = Roboflow(api_key="xtqdVslfhTMsizbzOtVG")
        project = rf.workspace().project("trash_balloon_detection-0vwqh")
        self.model = project.version('1').model
        '''
        path_to_pt = '/home/kimgracy/YOLO/YOLOv8/Trash_Balloon_Detection-1/runs/detect/train4/weights/best.pt'
        self.model = YOLO(path_to_pt)

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subcribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/rgb_raw/image_rect_color',
            self.image_callback,
            qos_profile
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            qos_profile
        )
        
        self.pcd_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.pcd_callback,
            qos_profile
        )

        # Create publishers
        self.yolo_publisher = self.create_publisher(YoloDetection, '/yolo_detection', qos_profile)

        self.depth_activation_publisher = self.create_publisher(DepthActivated, '/depth_activated', qos_profile)

        # Timer setup
        self.main_timer = self.create_timer(0.2, self.main_timer_callback)


    # services
    def draw_bboxes(self, bboxes):
        # Draw bounding boxes on RGB image
        color = (0, 255, 0)
        thickness = 3

        for row in bboxes :
            x_min, y_min, x_max, y_max, confidence, class_index = row.tolist()

            # Ensure coordinates are valid integers
            x_min, y_min, x_max, y_max = map(int, [x_min, y_min, x_max, y_max])

            label = 'balloon'
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1
            text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]
            text_x, text_y = int(x_min), int(y_min) - 10  # Position above the box
            text_color = (0, 255, 0)  # Same as rectangle color

            # Draw bounding box
            cv2.rectangle(self.raw_image, (x_min, y_min), (x_max, y_max), color, thickness)
            # Draw text box
            cv2.rectangle(self.raw_image, (text_x, text_y - text_size[1]), (text_x + text_size[0], text_y), text_color, -1)
            cv2.putText(self.raw_image, label, (text_x, text_y), font, font_scale, (0, 0, 0), font_thickness)

    '''
    def pixel_to_3d(self, pixel) :
        # convert pixel to 3d points
        u, v = pixel
        Z = self.depth_image[v, u]
        
        if Z <= 0 or Z == np.inf or np.isnan(Z):
            print(f"Invalid depth value at pixel ({u}, {v})")
            X = np.nan
            Y = np.nan

        else :
            X = Z * (u - self.cx) / self.fx
            Y = Z * (v - self.cy) / self.fy

        return (X,Y,Z)
    '''

    # Callback functions for subscribers
    def image_callback(self, msg):
        try:
            # rgb image -> OpenCV image
            image = CvBridge().imgmsg_to_cv2(msg, "rgb8")
            self.raw_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.height, self.width = self.raw_image.shape[:2]

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")


    def depth_callback(self, msg):
        try:
            # depth image -> OpenCV image
            self.depth_image = CvBridge().imgmsg_to_cv2(msg, "32FC1")

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    # pointcloud callback function - Unused
    '''
    def pcd_callback(self, msg):
        try:
            # ROS PointCloud2 -> Open3D PointCloud
            cloud_points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                cloud_points.append([point[0], point[1], point[2]])

            # Open3D PointCloud
            self.pcd = o3d.geometry.PointCloud()
            self.pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points, dtype=np.float64))

        except Exception as e:
            self.get_logger().error(f"Failed to convert PointCloud: {e}")
    '''

    def main_timer_callback(self):
        # print mean estimated value of a box
        # Show bounding boxes at OpenCV window
        # Publish YoloDetection & DepthActivated
        if self.depth_image is not None and self.raw_image is not None:

            results = self.model(self.raw_image)
            original_image = results[0].orig_img
            bboxes = results[0].boxes.data

            # publish DepthActivated msg
            depth_activated_msg = DepthActivated()

            if bboxes.size(0) == 0 :
                depth_activated_msg.depth_activated = False
            elif bboxes.size(0) != 0 :
                depth_activated_msg.depth_activated = True
            
            self.depth_activation_publisher.publish(depth_activated_msg)


            for bbox in bboxes:
                x_min, y_min, x_max, y_max, confidence, class_index = bbox.tolist()
                x_min, x_max, y_min, y_max = int(x_min), int(x_max), int(y_min), int(y_max)
                pixel_list = [(x_min, y_min), (x_min, y_max), (x_max, y_min), (x_max, y_max)]
                
                # print mean estimated depth value
                depth_values = self.depth_image[y_min:y_max, x_min:x_max]
                valid_depths = depth_values[np.isfinite(depth_values)]

                if valid_depths.size > 0 :
                    depth_value = np.mean(valid_depths)
                else :
                    depth_value = float('nan')

                class_name = "ladder"
                print(f"Estimated depth value of {class_name}: {depth_value:.2f}")

                # publish YoloDetection msg
                yolo_detection_msg = YoloDetection()
                yolo_detection_msg.label = class_name
                yolo_detection_msg.screen_width = float(640)
                yolo_detection_msg.screen_height = float(480)
                yolo_detection_msg.xmax = float(x_max)
                yolo_detection_msg.ymax = float(y_max)
                yolo_detection_msg.xmin = float(x_min)
                yolo_detection_msg.ymin = float(y_min)

                self.yolo_publisher.publish(yolo_detection_msg)

                
                # pixel to 3d coordinates
                point_list = []

                for pixel in pixel_list :
                    point_3d = self.pixel_to_3d(pixel=pixel)
                    point_list.append(point_3d)
                
                print(f"3d points for bounding box: {point_list}")


            # Draw bounding boxes and display images at OpenCV window
            self.draw_bboxes(results=results, bboxes=bboxes)
            cv2.imshow("Detection Results", original_image)

            cv2.waitKey(1)

            '''
            # Visualize 3d pointcloud
            o3d.visualization.draw_geometries([self.pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
            '''

        else:
            print("Image is None")



def main(args=None):
    rclpy.init(args=args)
    ros2_node = YOLO_depth()

    rclpy.spin(ros2_node)

    ros2_node.destroy_node()
    rclpy.shutdown()