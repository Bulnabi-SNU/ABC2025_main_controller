import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header  # ✅ Header 메시지 추가
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PCLPublisher(Node):
    def __init__(self):
        super().__init__('pcl_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 10)
        self.timer = self.create_timer(1.0, self.publish_pcl)  # 1초마다 퍼블리시

    def publish_pcl(self):
        """ 랜덤한 3D 포인트 클라우드 데이터를 퍼블리시 """
        points = np.random.uniform(-2, 2, (500, 3))  # -2m ~ 2m 사이의 랜덤 포인트 100개

        # ✅ Header 객체 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 현재 시간
        header.frame_id = "map"  # 프레임 ID

        # ✅ 올바르게 수정된 코드
        msg = pc2.create_cloud_xyz32(header, points)  # Header 객체 추가

        self.publisher.publish(msg)
        self.get_logger().info("📡 랜덤 포인트 클라우드 퍼블리시 완료!")

def main(args=None):
    rclpy.init(args=args)
    node = PCLPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()