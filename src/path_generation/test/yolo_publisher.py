import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import sys

class YOLOPublisher(Node):
    def __init__(self, goal_point):
        super().__init__('yolo_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/yolo_detection', 10)
        self.rviz_publisher = self.create_publisher(PointStamped, '/yolo_rviz', 10)
        self.timer = self.create_timer(2.0, self.publish_goal)  # 2초마다 퍼블리시

        # goal_point 설정 (X, Y, Z)
        self.goal_point = goal_point
        self.get_logger().info(f"🎯 목표 위치 설정됨: {self.goal_point}")

    def publish_goal(self):
        """ 목표 위치 퍼블리시 """
        msg = Float32MultiArray()
        msg.data = self.goal_point
        self.publisher.publish(msg)
        self.get_logger().info(f"📡 목표 위치 퍼블리시: {msg.data}")

        # RViz에서 시각화용 PointStamped 메시지 생성
        point_msg = PointStamped()
        point_msg.header.frame_id = "map"
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x, point_msg.point.y, point_msg.point.z = self.goal_point
        self.rviz_publisher.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)

    # 명령행 인자로 목표 좌표 받기
    if len(sys.argv) == 4:  # X, Y, Z 총 3개의 값이 있어야 함
        try:
            goal_point = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
        except ValueError:
            print("❌ 좌표는 숫자여야 합니다. 사용법: python3 yolo_publisher.py X Y Z")
            return
    else:
        goal_point = [-15.0, -2.0, 0.8]  # 기본값

    node = YOLOPublisher(goal_point)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
