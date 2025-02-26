#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros

class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(Image, '/depth_camera/depth_image', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.camera_info_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/depth_camera/points', 10)
        self.camera_info = None

        # TF Static Transform 브로드캐스터 생성 및 변환 브로드캐스트
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_transform()

    def broadcast_static_transform(self):
        # "map"에서 "depth_camera_link"로의 정적 변환 (여기서는 단순한 identity 변환 예시)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "depth_camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Broadcasted static transform from 'map' to 'depth_camera_link'.")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warning("Camera info not received yet.")
            return

        # Convert ROS Image to OpenCV image
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
            return

        # 카메라 내재 파라미터 추출
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        height, width = cv_depth.shape
        # 이미지 좌표 그리드 생성
        u = np.linspace(0, width - 1, width)
        v = np.linspace(0, height - 1, height)
        u, v = np.meshgrid(u, v)

        # 깊이 값을 실수형 배열로 변환 (단위: 미터)
        z = cv_depth.astype(np.float32)

        # 3D 좌표 계산: 원래 카메라 좌표 (x, y, z)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # 모든 픽셀의 좌표 배열 생성 (N x 3)
        points = np.stack((x, y, z), axis=2).reshape(-1, 3)
        # 유효한 포인트(깊이 값이 0보다 큰 경우)만 필터링
        valid = points[:, 2] > 0
        points = points[valid]

        # 좌표 변환: 현재 (x, y, z) -> (x, z, -y)
        # 즉, 두 번째와 세 번째 좌표를 교환하고, 세 번째 좌표에 부호 반전을 적용
        #points = np.column_stack((points[:, 0], points[:, 2], -points[:, 1]))
        points = np.column_stack((points[:, 0], points[:, 1], points[:, 2]))
        
        # PointCloud2 메시지 생성 및 퍼블리시 (header.frame_id는 그대로 사용하거나 필요에 따라 변경)
        header = msg.header
        header.frame_id = "depth_camera_link"  # TF 브로드캐스터와 일치시킴
        
        pc2_msg = pc2.create_cloud_xyz32(header, points.tolist())
        self.pc_pub.publish(pc2_msg)
        self.get_logger().debug(f"Published point cloud with {len(points)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
