import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import ActuatorServos
import time



class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        self.servo_flag = 1

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ActuatorServos 퍼블리셔 생성
        self.publisher_ = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', qos_profile)
        
        # 일정 간격으로 서보 신호 전송
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.send_servo_command)


    def send_servo_command(self):
        msg = ActuatorServos()
        msg.timestamp = int(time.time() * 1e6)  # 현재 타임스탬프 (마이크로초)
        msg.control = [float('nan')] * 8  # 기본값을 NaN으로 설정 (비활성화)
        
        # servo 1 제어
        msg.control[0] = 0.7 * self.servo_flag  # 모터 2번 70% 출력 (범위: -1 ~ 1)
        self.servo_flag *= -1

        self.publisher_.publish(msg)
        # self.get_logger().info(f'Sent Servo Command: {msg.control[1]} to Motor 2')



def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()