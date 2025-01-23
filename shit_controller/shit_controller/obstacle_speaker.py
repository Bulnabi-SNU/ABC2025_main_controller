# For SITL & debugging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from my_msgs.msg import YoloDetection, DepthActivated, VehicleState

class ObstacleSpeaker(Node):
    def __init__(self):
        super().__init__('obstacle_speaker')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.state =  'ready2flight'
        self.substate = 'before flight'

        # Create subscribers
        self.depth_activation_subscriber = self.create_subscription(DepthActivated, '/depth_activated', self.depth_activation_callback, qos_profile)
        self.vehicle_state_subscriber = self.create_subscription(VehicleState, '/vehicle_state', self.state_callback, qos_profile)

        # Create publishers
        self.publisher_ = self.create_publisher(YoloDetection, '/yolo_detection', qos_profile)

        self.time_counter = 0

        self.timer = self.create_timer(0.1, self.timer_callback)

    
    def timer_callback(self):
        if self.substate == 'rising':
            self.time_counter += 1

        if self.time_counter > 30:
            msg = YoloDetection()
            msg.label = "ladder"
            msg.screen_width = float(640)
            msg.screen_height = float(480)
            msg.xmax = float(200)
            msg.xmin = float(100)
            msg.ymax = float(200)
            msg.ymin = float(100)

            self.publisher_.publish(msg)

    def depth_activation_callback(self, msg):
        self.depth_activation = msg

    def state_callback(self, msg):
        # get vehicle state
        self.phase = msg.state
        self.subphase = msg.substate

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpeaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()