import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class YOLOPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/yolo_detection', 10)
        self.timer = self.create_timer(2.0, self.publish_goal)  # 3초마다 퍼블리시

    def publish_goal(self):
        """ 랜덤한 목표 위치 (풍선 위치) 퍼블리시 """
        msg = Float32MultiArray()
        ## msg.data = [random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0, 2)]  # X, Y, Z 좌표
        msg.data = [random.uniform(0,4),random.uniform(0,4),random.uniform(0,4)]
        self.publisher.publish(msg)
        self.get_logger().info(f"🎯 목표 위치 퍼블리시: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
