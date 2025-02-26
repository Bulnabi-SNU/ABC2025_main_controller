#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')
        self.get_logger().info("🚀 Drone Commander starting in Offboard mode")
        
        # 최신 웨이포인트(PoseStamped)를 저장할 변수
        self.current_setpoint = None
        
        # /planned_waypoint 토픽에서 PoseStamped 메시지 수신
        self.create_subscription(PoseStamped, '/planned_waypoint', self.waypoint_callback, 10)
        
        # PX4 Offboard 모드 제어를 위한 퍼블리셔 생성
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # 10Hz (0.1초 주기)로 메시지를 발행하는 타이머 생성
        self.timer = self.create_timer(0.1, self.publish_setpoint)

        # 오프보드 모드 및 ARM 활성화
        self.arm_vehicle()
        self.set_offboard_mode()

    def arm_vehicle(self):
        """드론을 Arm 상태로 변경"""
        self.get_logger().info("🛠 Arming the vehicle...")
        command = VehicleCommand()
        command.param1 = 1.0  # 1 = Arm, 0 = Disarm
        command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        self.vehicle_command_publisher.publish(command)

    def set_offboard_mode(self):
        """Offboard 모드 활성화"""
        self.get_logger().info("🛰 Switching to Offboard mode...")
        command = VehicleCommand()
        command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command.param1 = 1.0  # Main mode: PX4_CUSTOM_MAIN_MODE_OFFBOARD
        command.param2 = 6.0  # Sub mode: PX4_CUSTOM_SUB_MODE_AUTO_LOITER
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        self.vehicle_command_publisher.publish(command)

    def waypoint_callback(self, msg):
        """ 새로운 웨이포인트를 수신하면 저장 """
        self.get_logger().info(f"📍 New waypoint received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}")
        self.current_setpoint = msg

    def publish_setpoint(self):
        """ OffboardControlMode 및 Waypoint를 주기적으로 퍼블리시 """
        # OffboardControlMode 설정 (Position Control 사용)
        control_mode = OffboardControlMode()
        control_mode.position = True
        control_mode.velocity = False
        control_mode.acceleration = False
        control_mode.attitude = False
        control_mode.body_rate = False
        control_mode.timestamp = self.get_clock().now().nanoseconds // 1000  # PX4 expects timestamp in microseconds
        self.offboard_mode_publisher.publish(control_mode)

        # 최신 Waypoint가 없다면 발행하지 않음
        if self.current_setpoint is None:
            return

        # setpoint 메시지 발행 (PX4의 TrajectorySetpoint 사용)
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [
            self.current_setpoint.pose.position.x,
            self.current_setpoint.pose.position.y,
            self.current_setpoint.pose.position.z
        ]
        trajectory_msg.yaw = 0.0  # 기본적으로 0도 유지
        trajectory_msg.timestamp = self.get_clock().now().nanoseconds // 1000  # PX4 expects timestamp in microseconds

        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().debug(f"📡 Published Offboard Setpoint: x={trajectory_msg.position[0]:.2f}, y={trajectory_msg.position[1]:.2f}, z={trajectory_msg.position[2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
