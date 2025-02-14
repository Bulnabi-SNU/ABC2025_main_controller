#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d  # Open3D를 사용하여 PCD 파일 로드
import struct
from std_msgs.msg import Header  # Header 메시지 추가

class PCDPublisher(Node):
    def __init__(self, pcd_file):
        super().__init__('pcd_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/custom_fused_cloud', 10)
        self.timer = self.create_timer(1.0, self.publish_pcd)  # 1초마다 퍼블리시
        self.pcd_file = pcd_file
        self.pcd_data = self.load_pcd(self.pcd_file)
        self.get_logger().info(f"🚀 PCD Publisher 노드가 실행되었습니다! 파일: {pcd_file}")

    def load_pcd(self, file_path):
        """ Open3D를 사용하여 PCD 파일을 로드하고 Numpy 배열로 변환 """
        try:
            pcd = o3d.io.read_point_cloud(file_path)
            points = np.asarray(pcd.points)
            self.get_logger().info(f"✅ PCD 파일 로드 완료! 점 개수: {len(points)}")
            return points
        except Exception as e:
            self.get_logger().error(f"❌ PCD 파일 로드 실패: {e}")
            return None

    def publish_pcd(self):
        """ PCD 데이터를 PointCloud2 메시지로 변환하여 퍼블리시 """
        if self.pcd_data is None:
            self.get_logger().error("⚠️ PCD 데이터가 비어 있습니다. 퍼블리시할 수 없습니다.")
            return

        # 올바른 Header 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 현재 시간 설정
        header.frame_id = "map"  # 좌표계 설정 (예: "map" 또는 "base_link")

        # PointCloud2 메시지 생성
        cloud_msg = pc2.create_cloud_xyz32(header, self.pcd_data)

        # 퍼블리시
        self.publisher.publish(cloud_msg)
        self.get_logger().info(f"📡 PCD 데이터 퍼블리시 완료! 점 개수: {len(self.pcd_data)}")

def main(args=None):
    rclpy.init(args=args)
    pcd_file_path = "./assets/pointcloud_0214.pcd"  # 🚨 여기에 PCD 파일 경로 입력!
    node = PCDPublisher(pcd_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
