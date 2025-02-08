import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import random

class YOLOPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/yolo_detection', 10)
        self.rviz_publisher = self.create_publisher(PointStamped, '/yolo_rviz', 10)
        self.timer = self.create_timer(2.0, self.publish_goal)  # 3초마다 퍼블리시

    def publish_goal(self):
        """ 랜덤한 목표 위치 (풍선 위치) 퍼블리시 """
        msg = Float32MultiArray()
        ## msg.data = [random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0, 2)]  # X, Y, Z 좌표
        # msg.data = [random.uniform(0,10),random.uniform(0,10),random.uniform(0,10)]
        msg.data = [-5.0, 0.0, 0.8]
        self.publisher.publish(msg)
        self.get_logger().info(f"🎯 목표 위치 퍼블리시: {msg.data}")

        point_msg = PointStamped()
        point_msg.header.frame_id = "map"
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = msg.data[0]
        point_msg.point.y = msg.data[1]
        point_msg.point.z = msg.data[2]
        self.rviz_publisher.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
