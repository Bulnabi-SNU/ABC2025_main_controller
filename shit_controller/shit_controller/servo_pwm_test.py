import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import ActuatorServos, ActuatorMotors
import time




class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        self.servo_flag = 1
        self.motor_flag = 1

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ActuatorServos 퍼블리셔 생성
        self.servo_actuator_publisher = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', qos_profile)
        self.motor_actuator_publisher = self.create_publicher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        
        # 일정 간격으로 서보 신호 전송
        self.actuator_time_period = 1.0
        self.time_period = 0.01
        self.servo_timer = self.create_timer(self.actuator_time_period, self.send_servo_command)
        self.motor_timer = self.create_timer(self.actuator_time_period, self.send_motor_command)
        self.offboard_heartbeat = self.create_timer(self.time_period, self.offboard_heartbeat_callback)


    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(position=True)


    def send_servo_command(self):
        msg = ActuatorServos()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)
        msg.control = [float('nan')] * 8  # 기본값을 NaN으로 설정 (비활성화)
        self.servo_flag *= -1

        # servo
        for i in range(0,8):
            msg.control[i] = 0.7 * self.servo_flag  # 모터 2번 70% 출력 (범위: -1 ~ 1)

        self.servo_actuator_publisher.publish(msg)
        # self.get_logger().info(f'Sent Servo Command: {msg.control[1]} to Motor 2')
    
    def send_motor_command(self):
        msg = ActuatorMotors()
        msg.reversible_flags = 0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)
        msg.control = [float('nan')] * 12  # 기본값을 NaN으로 설정 (비활성화)
        self.motor_flag *= -1

        # servo
        for i in range(0,12):
            msg.control[i] = 0.7 * self.motor_flag  # 모터 2번 70% 출력 (범위: -1 ~ 1)

        self.motor_actuator_publisher.publish(msg)
    
    
    def publish_offboard_control_mode(self, **kwargs):
        msg = OffboardControlMode()
        msg.position = kwargs.get("position", False)
        msg.velocity = kwargs.get("velocity", False)
        msg.acceleration = kwargs.get("acceleration", False)
        msg.attitude = kwargs.get("attitude", False)
        msg.body_rate = kwargs.get("body_rate", False)
        msg.thrust_and_torque = kwargs.get("thrust_and_torque", True)
        msg.direct_actuator = kwargs.get("direct_actuator", True)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()