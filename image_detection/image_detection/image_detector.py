# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import sensor_msgs
from sensor_msgs.msg import Image

# import other libraries
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


class RawImageNode(Node):

    def __init__(self):

        super().__init__('image_detection')

        # Initialize Variables
        self.bridge = CvBridge()
        self.raw_image = None
        self.selected_point = None

        # Set QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subcribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile
        )

        cv2.namedWindow("Raw Image")
        cv2.setMouseCallback("Raw Image", self.select_point)

        # Create Timers
        self.main_timer = self.create_timer(0.1, self.main_timer_callback)


    def select_point(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_point = (x, y)
            self.get_logger().info(f"Point selected: ({x}, {y})")


    def image_callback(self, msg):
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")


    def main_timer_callback(self):
        if self.raw_image is not None:
            cv2.imshow("Raw Image", self.raw_image)

            if self.selected_point is not None:
                
                x, y = self.selected_point

                if 0 <= y < self.raw_image.shape[0] and 0 <= x < self.raw_image.shape[1]:
                    rgb_value = self.raw_image[y, x]
                    self.get_logger().info(f"RGB value at ({x}, {y}): {rgb_value}")

                else:
                    self.get_logger().warning("Selected point is out of bounds.")

            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    raw_image = RawImageNode()

    rclpy.spin(raw_image)

    raw_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)