#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from px4_msgs.msg import TrajectorySetpoint  # PX4 ROS2 메시지 패키지
from geometry_msgs.msg import PoseStamped
import numpy as np

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')
        # dstar_bezier_path_planner 노드가 퍼블리시하는 경로 토픽 구독
        self.subscription = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # PX4로 명령(setpoint)을 발행하는 퍼블리셔
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        
        # 20Hz (0.05초 주기)로 setpoint 발행을 위한 타이머
        self.timer_period = 0.05  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 경로 데이터를 저장할 변수 (PoseStamped 리스트)
        self.path_points = []
        self.current_point_idx = 0
        
        self.get_logger().info("Drone Commander 노드가 시작되었습니다.")

    def path_callback(self, msg: Path):
        """ dstar_bezier_path_planner가 퍼블리시한 경로를 수신 """
        num_points = len(msg.poses)
        self.get_logger().info(f"새로운 경로 수신 (포인트 수: {num_points})")
        if num_points == 0:
            self.get_logger().warn("경로에 포인트가 없습니다!")
            return
        
        self.path_points = msg.poses
        self.current_point_idx = 0

    def timer_callback(self):
        """ 경로에 포함된 포인트를 순차적으로 PX4로 발행 """
        if not self.path_points:
            return

        # 경로의 마지막 포인트에 도달하면 마지막 포인트를 계속 사용
        if self.current_point_idx >= len(self.path_points):
            current_pose = self.path_points[-1]
        else:
            current_pose = self.path_points[self.current_point_idx]
            self.current_point_idx += 1
        
        # PoseStamped의 position에서 좌표 추출
        pos = current_pose.pose.position
        if isinstance(pos, (list, tuple, np.ndarray)):
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        else:
            x, y, z = pos.x, pos.y, pos.z

        # TrajectorySetpoint 메시지 생성 (PX4 offboard 제어용)
        setpoint_msg = TrajectorySetpoint()
        # PX4의 TrajectorySetpoint 메시지에서 position은 배열로 정의되어 있으므로,
        # 리스트로 직접 할당합니다.
        setpoint_msg.position = [x, y, z]
        setpoint_msg.yaw = 0.0  # 필요 시 적절한 yaw 값 계산

        self.setpoint_pub.publish(setpoint_msg)
        self.get_logger().debug(
            f"Setpoint 발행: ({x:.2f}, {y:.2f}, {z:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
