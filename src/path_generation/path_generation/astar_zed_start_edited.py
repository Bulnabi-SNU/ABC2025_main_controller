#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped
import numpy as np
import heapq
import sensor_msgs_py.point_cloud2 as pc2
import sys
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition

# 무한대 상수
INF = float('inf')

class AStarPathPlanner(Node):
    def __init__(self, parser_mode):
        super().__init__('astar_path_planner')

        # -------------------------------------------
        # Getting the current position of the drone
        # -------------------------------------------

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_status = VehicleStatus()

        # vehicle position, velocity, and yaw
        self.pos = np.array([0.0, 0.0, 0.0])        # local
        self.pos_gps = np.array([0.0, 0.0, 0.0])    # global
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.home_position = None
        self.controller_time_period = 0.05          # 20Hz. same with the controller

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )


        # TF 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 사용자가 지정한 토픽 선택
        if parser_mode == "custom":
            pointcloud_topic = "/custom_fused_cloud"
        else:
            pointcloud_topic = "/zed/zed_node/mapping/fused_cloud"

        self.create_subscription(PointCloud2, pointcloud_topic, self.pcl_callback, 10)
        self.create_subscription(Float32MultiArray, '/yolo_detection', self.balloon_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # 위치 모니터링을 위한 타이머 추가 (1Hz)
        self.create_timer(1, self.monitor_position)
        
        # 이전 위치를 저장할 변수 추가
        self.previous_position = None
        self.position_threshold = 0.3  # 30cm 이상 이동 시 경로 재계획
        
        # 격자 맵 관련 변수
        self.occupancy_grid = None
        self.grid_origin = None
        self.resolution = None

        # 목표 월드 좌표
        self.goal_pos_world = None

        # 바닥과 천장 z좌표를 저장할 변수
        self.floor_z = None
        self.ceiling_z = None

        self.get_logger().info("🚀 AStarPathPlanner 실행됨")

    def monitor_position(self):
        """카메라 위치를 주기적으로 모니터링하고 필요시 경로를 재계획합니다."""
        if self.occupancy_grid is None or self.goal_pos_world is None:
            return

        current_pos = self.get_start_position()
        if current_pos is None:
            return

        # 처음 위치가 설정되는 경우
        if self.previous_position is None:
            self.previous_position = current_pos
            return

        # 이전 위치와의 거리 계산
        distance = np.sqrt(sum((np.array(current_pos) - np.array(self.previous_position)) ** 2))
        
        # 지정된 threshold 이상 이동했을 경우 경로 재계획
        if distance > self.position_threshold:
            self.get_logger().info(f"📏 카메라가 {distance:.2f}m 이동했습니다. 경로를 재계획합니다.")
            self.previous_position = current_pos
            self.run_astar()

    def get_start_position(self):
        """zed_camera_center의 현재 위치를 가져옵니다."""
        try:
            # 최신 transform을 가져오기 위해 시도
            transform = self.tf_buffer.lookup_transform(
                'map',
                'zed_camera_center',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            
            start_pos = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            return start_pos
            
        except Exception as e:
            # 최신 transform 가져오기 실패 시, 가장 최근 transform을 시도
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'zed_camera_center',
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.0)
                )
                
                start_pos = [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]
                return start_pos
                
            except Exception as e2:
                self.get_logger().error(f"🚨 TF 조회 실패: {str(e2)}")
                return None

    def find_floor_ceiling(self, points):
        """포인트 클라우드에서 바닥과 천장의 z좌표를 찾습니다."""
        z_coords = points[:, 2]  # z좌표만 추출
        
        # 히스토그램 생성
        hist, bin_edges = np.histogram(z_coords, bins=50)
        
        # 노이즈 제거를 위한 스무딩
        smoothed_hist = np.convolve(hist, np.ones(3)/3, mode='valid')
        
        # 로컬 맥시마 찾기
        peaks = []
        for i in range(1, len(smoothed_hist)-1):
            if smoothed_hist[i] > smoothed_hist[i-1] and smoothed_hist[i] > smoothed_hist[i+1]:
                peaks.append((smoothed_hist[i], bin_edges[i]))
        
        # 가장 큰 두 개의 피크 찾기
        peaks.sort(reverse=True)
        if len(peaks) >= 2:
            z1, z2 = peaks[0][1], peaks[1][1]
            self.floor_z = min(z1, z2)
            self.ceiling_z = max(z1, z2)
            
            # 안전 마진 추가 (10cm)
            self.floor_z += 0.1
            self.ceiling_z -= 0.1
            
            self.get_logger().info(f"🏠 바닥/천장 감지 - 바닥: {self.floor_z:.2f}m, 천장: {self.ceiling_z:.2f}m")
        else:
            self.get_logger().warning("⚠️ 바닥과 천장을 감지할 수 없습니다!")

    def pcl_callback(self, msg):
        self.get_logger().info("📡 포인트 클라우드 데이터 수신!")
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if points.size == 0:
            self.get_logger().warning("⚠️ 수신된 포인트 클라우드가 비어 있음!")
            return
            
        # 바닥과 천장 감지
        self.find_floor_ceiling(points)

        desired_resolution = 0.1  # 10cm 해상도
        self.occupancy_grid, self.grid_origin, self.resolution = self.create_occupancy_grid(points, desired_resolution)
        self.get_logger().info(f"📊 Occupancy Grid 생성 완료, shape: {self.occupancy_grid.shape}")

        if self.goal_pos_world:
            self.run_astar()

    def create_occupancy_grid(self, points, resolution):
        """실제 point cloud 범위를 기반으로 occupancy grid 생성"""
        # Point cloud의 실제 범위 계산
        min_coords = np.min(points, axis=0) - resolution  # 여유 공간 추가
        max_coords = np.max(points, axis=0) + resolution  # 여유 공간 추가
        
        grid_shape = np.ceil((max_coords - min_coords) / resolution).astype(int) + 1
        grid = np.zeros(tuple(grid_shape), dtype=np.uint8)
        
        obstacle_radius = max(1, int(0.1 / resolution))
        
        # 인덱스 계산을 한 번만 수행
        indices = ((points - min_coords) / resolution).astype(int)
        
        # 유효한 인덱스만 필터링
        valid_mask = np.all((indices >= 0) & (indices < grid_shape), axis=1)
        valid_indices = indices[valid_mask]
        
        # 벡터화된 연산으로 장애물 영역 설정
        for idx in valid_indices:
            i, j, k = idx
            i_min, i_max = max(0, i - obstacle_radius), min(grid_shape[0], i + obstacle_radius + 1)
            j_min, j_max = max(0, j - obstacle_radius), min(grid_shape[1], j + obstacle_radius + 1)
            k_min, k_max = max(0, k - obstacle_radius), min(grid_shape[2], k + obstacle_radius + 1)
            grid[i_min:i_max, j_min:j_max, k_min:k_max] = 1
        
        return grid, min_coords, resolution
    
    def world_to_grid(self, pos_world):
        pos_world = np.array(pos_world)
        idx = ((pos_world - self.grid_origin) / self.resolution).astype(int)
        return tuple(idx)

    def grid_to_world(self, idx):
        idx = np.array(idx)
        pos_world = self.grid_origin + (idx + 0.5) * self.resolution
        return pos_world.tolist()

    def balloon_callback(self, msg):
        self.goal_pos_world = [round(msg.data[0], 3), round(msg.data[1], 3), round(msg.data[2], 3)]
        self.get_logger().info(f"🎯 목표 지점 업데이트 (월드 좌표): {self.goal_pos_world}")

        if self.occupancy_grid is None:
            self.get_logger().warning("⚠️ 아직 occupancy grid가 생성되지 않음")
            return

        self.run_astar()

    def publish_start_point(self):
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "map"
        point_msg.point.x = float(self.start_pos_world[0])
        point_msg.point.y = float(self.start_pos_world[1])
        point_msg.point.z = float(self.start_pos_world[2])
        self.start_point_pub.publish(point_msg)

    def get_neighbors(self, node):
        i, j, k = node
        directions = [
            (1, 0, 0), (-1, 0, 0),  # x 방향
            (0, 1, 0), (0, -1, 0),  # y 방향
            (0, 0, 1), (0, 0, -1)   # z 방향
        ]
        neighbors = []
        
        # z좌표의 실제 월드 좌표 계산을 위한 변환
        for di, dj, dk in directions:
            neighbor = (i + di, j + dj, k + dk)
            # 새로운 z 위치의 월드 좌표 계산
            new_world_z = self.grid_origin[2] + ((k + dk) * self.resolution)
            
            if (0 <= neighbor[0] < self.occupancy_grid.shape[0] and
                0 <= neighbor[1] < self.occupancy_grid.shape[1] and
                0 <= neighbor[2] < self.occupancy_grid.shape[2] and
                self.floor_z <= new_world_z <= self.ceiling_z and  # 바닥/천장 제한
                self.occupancy_grid[neighbor] == 0):  # 장애물이 아닌 경우만 포함
                neighbors.append(neighbor)
        
        return neighbors

    def heuristic(self, a, b):
        return np.sqrt(sum((np.array(a) - np.array(b)) ** 2))

    def astar(self, start, goal):
        frontier = [(0, start)]
        self.came_from = {start: None}  # came_from를 클래스 변수로 저장
        cost_so_far = {start: 0}
        
        max_iterations = 50000
        iterations = 0

        while frontier and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.resolution

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    self.came_from[next_node] = current

            # 100번째 반복마다 현재까지의 경로 발행
            if iterations % 100 == 0:
                partial_path = self.reconstruct_path(self.came_from, current)
                if partial_path:
                    self.publish_path(partial_path, is_final=False)

        if iterations >= max_iterations:
            self.get_logger().error("🚨 최대 반복 횟수 초과!")
            return None

        if goal not in self.came_from:
            self.get_logger().error("🚨 경로를 찾을 수 없습니다!")
            return None

        return self.reconstruct_path(self.came_from, goal)

    def reconstruct_path(self, came_from, current):
        path = []
        while current is not None:
            path.append(self.grid_to_world(current))
            current = came_from.get(current)
        return path[::-1]  # 역순으로 반환

    def run_astar(self):
        # 현재 카메라 위치를 시작점으로 가져옴
        start_pos_world = self.get_start_position()
        if start_pos_world is None:
            self.get_logger().error("🚨 시작 위치를 가져올 수 없습니다!")
            return

        start_idx = self.world_to_grid(start_pos_world)
        goal_idx = self.world_to_grid(self.goal_pos_world)

        try:
            if self.occupancy_grid[start_idx] == 1:
                self.get_logger().error("🚨 출발 지점이 장애물 내부에 있습니다!")
                return
            if self.occupancy_grid[goal_idx] == 1:
                self.get_logger().error("🚨 목표 지점이 장애물 내부에 있습니다!")
                return

            self.get_logger().info("📌 A* 경로 탐색 시작")
            path = self.astar(start_idx, goal_idx)
            
            if path:
                # 먼저 원본 경로 발행
                # self.publish_path(path, is_final=False)
                
                # 스무딩된 최종 경로 발행
                try:
                    smoothed_path = self.smooth_path(path)
                    self.publish_path(smoothed_path, is_final=True)
                    self.get_logger().info("✅ 경로 계획 완료")
                except Exception as e:
                    self.get_logger().error(f"🚨 경로 스무딩 중 오류 발생: {str(e)}")
            else:
                self.get_logger().warning("⚠️ 경로를 찾지 못했습니다.")
        except Exception as e:
            self.get_logger().error(f"🚨 경로 계획 중 오류 발생: {str(e)}")

    def bezier_curve(self, points, num_points=50):
        """주어진 제어점들로 베지에 커브를 생성합니다."""
        def pascal_triangle(n):
            line = [1]
            for k in range(n):
                line.append(int(line[k] * (n - k) / (k + 1)))
            return line

        def bernstein_poly(i, n, t):
            coef = pascal_triangle(n)[i]
            return coef * (t ** i) * ((1 - t) ** (n - i))

        points = np.array(points, dtype=np.float64)  # 명시적으로 float64 타입으로 변환
        n = len(points) - 1
        t = np.linspace(0, 1, num_points)
        curve = np.zeros((num_points, 3), dtype=np.float64)
        
        for i in range(n + 1):
            b = bernstein_poly(i, n, t)
            b = b.reshape(-1, 1)  # (num_points, 1) 형태로 변환
            curve += np.tile(points[i], (num_points, 1)) * b
        
        return curve

    def smooth_path(self, path, target_speed=4.0, max_contr8ol_points=1000):
        """A* 경로를 베지에 커브를 사용하여 부드럽게 만듭니다."""
        if len(path) < 2:
            return path
        
        # 목적지까지의 거리 계산
        start = np.array(path[0])
        goal = np.array(path[-1])
        distance = np.linalg.norm(goal - start)  # 유클리드 거리 계산
        # 목표 속도에 따른 베지에 점 개수 계산
        total_time = distance / target_speed
        num_bezier_points = max(4, min(int(total_time / self.controller_time_period), 1000))

        control_points = min(4, len(path))

        # 첫 번째 이동 위치 계산
        shift = (total_time / num_bezier_points) * self.vel
        first_point = start + shift

        # 초기 control_path 설정
        control_path = [start, first_point]
        filtered_points = [p for p in path if np.linalg.norm(p - start) > np.linalg.norm(shift)]
        if len(filtered_points) > 2:
            # 필터링 된 포인트를 전부 쓸까 아니면 일부만 쓸까?
            indices = np.linspace(0, len(filtered_points) - 1, min(control_points, num_bezier_points-len(control_path)), dtype=int)
            control_path.extend([filtered_points[i] for i in indices])
        
        try:
            # 베지에 커브 생성
            smoothed_points = self.bezier_curve(control_path, num_points=num_bezier_points)
            
            # 장애물 체크
            is_safe = True
            for point in smoothed_points:
                grid_idx = self.world_to_grid(point)
                if not (0 <= grid_idx[0] < self.occupancy_grid.shape[0] and
                    0 <= grid_idx[1] < self.occupancy_grid.shape[1] and
                    0 <= grid_idx[2] < self.occupancy_grid.shape[2]):
                    is_safe = False
                    break
                if self.occupancy_grid[grid_idx] == 1:
                    is_safe = False
                    break
            
            if is_safe:
                self.get_logger().info(f"✅ 안전한 스무딩 경로 생성 완료 (제어점: {control_points}개)")
                return smoothed_points.tolist()
            
        except Exception as e:
            self.get_logger().warning(f"⚠️ 스무딩 중 오류 발생: {str(e)}")
    
        # 안전한 경로를 찾지 못했거나 오류가 발생한 경우 원래 경로 반환
        self.get_logger().warning("⚠️ 안전한 스무딩 경로를 찾지 못해 원본 경로를 사용합니다.")
        return path

    def publish_path(self, path, is_final=False):
        """경로를 발행합니다."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Convert path points to PoseStamped messages
        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position = Point(x=float(point[0]), 
                                    y=float(point[1]), 
                                    z=float(point[2]))
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


    """
    PX4 callback functions
    """
    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg
    
    def vehicle_local_position_callback(self, msg):
        if self.home_position is None:
            self.home_position = np.array([msg.x, msg.y, msg.z])
        
        self.vehicle_local_position = msg
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.pos = self.pos - self.home_position
        self.vel = np.array([msg.vx, msg.vy, msg.vz])
        self.yaw = msg.heading



def main(args=None):
    rclpy.init(args=args)
    parser_mode = sys.argv[1] if len(sys.argv) > 1 else "zed"

    if parser_mode not in ["zed", "custom"]:
        print("❌ 잘못된 인자! 사용법: ros2 run path_generation astar_path_planner [NONE/custom]")
        return

    node = AStarPathPlanner(parser_mode)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()