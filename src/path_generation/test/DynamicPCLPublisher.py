a#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class DynamicPCLPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_pcl_publisher')
        # 실제 zed_ros_wrapper와 같은 토픽 이름으로 퍼블리시 (dstar_bezier_path_planner가 구독)
        self.publisher = self.create_publisher(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 10)
        # 주기: 0.5초마다 업데이트 (원하는 주기로 조정 가능)
        self.timer = self.create_timer(0.5, self.publish_pcl)
        
        # 로봇의 현재 위치 (예시로 x축 방향으로만 이동)
        self.robot_position = np.array([0.0, 0.0, 0.0])

    def publish_pcl(self):
        # 시뮬레이션: 매 콜백마다 로봇이 전방으로 0.1m 이동 (원하는 속도로 조절 가능)
        self.robot_position[0] += 0.1

        # 전방 영역 설정: 로봇의 x 위치부터 x+5m, y는 현재 위치 기준 ±2m, z는 0~2m 영역
        x_min = self.robot_position[0]
        x_max = self.robot_position[0] + 5.0
        y_min = self.robot_position[1] - 2.0
        y_max = self.robot_position[1] + 2.0
        z_min = 0.0
        z_max = 2.0

        # 예시: 이 영역 내에서 무작위로 num_points 개의 점 생성 (실제 환경에서는 센서 데이터가 들어옴)
        num_points = 1000
        points = np.random.uniform(low=[x_min, y_min, z_min],
                                   high=[x_max, y_max, z_max],
                                   size=(num_points, 3))
        
        # Header 생성: 프레임 ID는 "map" 또는 원하는 좌표계로 지정
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # PointCloud2 메시지 생성 (pc2.create_cloud_xyz32는 float32 기반 메시지를 생성)
        msg = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Published point cloud for region ahead of robot at x = {self.robot_position[0]:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPCLPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
