#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import heapq
import sensor_msgs_py.point_cloud2 as pc2
import sys

# 무한대 상수
INF = float('inf')

class DStarLitePathPlanner(Node):
    def __init__(self, parser_mode):
        super().__init__('dstar_lite_path_planner')

        # 사용자가 지정한 토픽 선택
        if parser_mode == "custom":
            pointcloud_topic = "/custom_fused_cloud"
        else:
            pointcloud_topic = "/zed/zed_node/mapping/fused_cloud"

        self.create_subscription(PointCloud2, pointcloud_topic, self.pcl_callback, 10)
        self.create_subscription(Float32MultiArray, '/yolo_detection', self.balloon_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # 격자 맵 관련 변수 (월드 좌표 -> 격자 좌표 변환)
        self.occupancy_grid = None    # 3D numpy array (0: free, 1: obstacle)
        self.grid_origin = None       # occupancy grid의 최소 좌표 (월드 좌표 기준)
        self.resolution = None        # 격자 해상도 (m)

        # 로봇 및 목표 월드 좌표 (예: 드론, 풍선)
        self.start_pos_world = [0, 0, 0]  # 월드 좌표
        self.goal_pos_world = None        # 월드 좌표, balloon_callback에서 업데이트

        # D* Lite 관련 변수 (격자 인덱스 기준)
        self.g = {}       # key: (i, j, k) 격자 인덱스, value: 비용
        self.rhs = {}     # key: (i, j, k), value: one-step lookahead 값
        self.km = 0
        self.open_list = []  # 우선순위 큐 (heapq)

        self.get_logger().info("🚀 DStarLitePathPlanner 실행됨")

    # --- 포인트 클라우드 관련 함수 ---
    def pcl_callback(self, msg):
        self.get_logger().info("📡 포인트 클라우드 데이터 수신!")
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if points.size == 0:
            self.get_logger().warning("⚠️ 수신된 포인트 클라우드가 비어 있음! occupancy grid를 생성하지 않음.")
            return

        desired_resolution = 0.05  # 5cm 해상도
        self.occupancy_grid, self.grid_origin, self.resolution = self.create_occupancy_grid(points, desired_resolution)
        self.get_logger().info(f"📊 Occupancy Grid 생성 완료, shape: {self.occupancy_grid.shape}")

        # 장애물 업데이트 후, 목표가 설정되어 있다면 경로 재계산
        if self.goal_pos_world:
            self.run_dstar_lite()

    def create_occupancy_grid(self, points, resolution):
        """ 
        points: numpy array (N x 3) 월드 좌표
        resolution: 격자 해상도 (m)
        
        반환: occupancy_grid (3D numpy array), grid_origin (최소 월드 좌표)
        """
        min_coords = np.array([-20, -20, -5])
        max_coords = np.array([20, 20, 5])
        grid_shape = np.ceil((max_coords - min_coords) / resolution).astype(int) + 1

        # 3D occupancy grid 초기화 (0: free)
        grid = np.zeros(tuple(grid_shape), dtype=np.uint8)

        # 장애물 확장 반경 (30cm/0.05m -> 6격자)
        obstacle_radius = int(0.01 / resolution)

        # 각 포인트를 격자 인덱스로 변환하고 장애물로 표시 (1)
        for p in points:
            idx = ((p - min_coords) / resolution).astype(int)
            if (idx < 0).any() or (idx >= grid_shape).any():
                continue

            # 30cm 반경 내 격자들을 장애물로 설정
            for di in range(-obstacle_radius, obstacle_radius + 1):
                for dj in range(-obstacle_radius, obstacle_radius + 1):
                    for dk in range(-obstacle_radius, obstacle_radius + 1):
                        neighbor = idx + np.array([di, dj, dk])
                        if (neighbor < 0).any() or (neighbor >= grid_shape).any():
                            continue
                        grid[tuple(neighbor)] = 1  # 장애물로 설정

        return grid, min_coords, resolution

    def world_to_grid(self, pos_world):
        """ 월드 좌표를 occupancy grid의 인덱스로 변환 """
        pos_world = np.array(pos_world)
        idx = ((pos_world - self.grid_origin) / self.resolution).astype(int)
        return tuple(idx)

    def grid_to_world(self, idx):
        """ 격자 인덱스를 월드 좌표의 중앙으로 변환 """
        idx = np.array(idx)
        pos_world = self.grid_origin + (idx + 0.5) * self.resolution
        return pos_world.tolist()

    # --- 목표 관련 함수 ---
    def balloon_callback(self, msg):
        """ 풍선 위치를 받아 목표 월드 좌표로 업데이트 후 경로 생성 """
        self.goal_pos_world = [round(msg.data[0], 3), round(msg.data[1], 3), round(msg.data[2], 3)]
        self.get_logger().info(f"🎯 목표 지점 업데이트 (월드 좌표): {self.goal_pos_world}")

        if self.occupancy_grid is None:
            self.get_logger().warning("⚠️ 아직 occupancy grid가 생성되지 않음")
            return

        self.run_dstar_lite()

    # --- D* Lite 알고리즘 관련 함수 (격자 좌표 기준) ---
    def calculate_key(self, s):
        """ 
        s: 격자 인덱스 (튜플)
        key(s) = [min(g(s), rhs(s)) + h(start, s) + km, min(g(s), rhs(s))]
        """
        g_val = self.g.get(s, INF)
        rhs_val = self.rhs.get(s, INF)
        k1 = min(g_val, rhs_val) + self.heuristic(self.start_idx, s) + self.km
        k2 = min(g_val, rhs_val)
        return (k1, k2)

    def update_vertex(self, s):
        if s != self.goal_idx:
            min_val = INF
            for sp in self.get_neighbors(s):
                c = self.cost(s, sp)
                g_sp = self.g.get(sp, INF)
                if c + g_sp < min_val:
                    min_val = c + g_sp
            self.rhs[s] = min_val

        if self.g.get(s, INF) != self.rhs.get(s, INF):
            heapq.heappush(self.open_list, (self.calculate_key(s), s))
            self.get_logger().debug(f"🔄 update_vertex: {s}, g: {self.g.get(s, INF)}, rhs: {self.rhs.get(s, INF)}")

    def compute_shortest_path(self):
        max_iterations = 50000  # 최대 반복 횟수 제한
        iteration = 0

        while (self.open_list and (self.open_list[0][0] < self.calculate_key(self.start_idx))) or \
            (self.rhs.get(self.start_idx, INF) != self.g.get(self.start_idx, INF)):

            if iteration > max_iterations:
                self.get_logger().error("🚨 D* Lite 알고리즘이 50000번 이상 반복됨. 무한 루프 가능성 있음!")
                break

            k_old, u = heapq.heappop(self.open_list)
            k_new = self.calculate_key(u)
            if k_old < k_new:
                heapq.heappush(self.open_list, (k_new, u))
            elif self.g.get(u, INF) > self.rhs.get(u, INF):
                self.g[u] = self.rhs.get(u, INF)
                for s in self.get_neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = INF
                self.update_vertex(u)
                for s in self.get_neighbors(u):
                    self.update_vertex(s)

            iteration += 1

    def initialize_dstar(self):
        """ 모든 노드 초기화 및 목표 노드 설정 (격자 인덱스 기준) """
        self.g.clear()
        self.rhs.clear()
        self.open_list.clear()
        self.km = 0

        self.start_idx = self.world_to_grid(self.start_pos_world)
        self.goal_idx  = self.world_to_grid(self.goal_pos_world)

        self.g[self.goal_idx] = INF
        self.rhs[self.goal_idx] = 0
        heapq.heappush(self.open_list, (self.calculate_key(self.goal_idx), self.goal_idx))
        self.get_logger().debug(f"초기화 완료: goal {self.goal_idx}의 rhs=0, open_list에 추가")

    def get_neighbors(self, s):
        """ 격자 상에서 6방향 (상하좌우, 전후)의 이웃 반환 """
        i, j, k = s
        directions = [
            (1, 0, 0), (-1, 0, 0),
            (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1)
        ]
        neighbors = []
        for di, dj, dk in directions:
            neighbor = (i + di, j + dj, k + dk)
            if (0 <= neighbor[0] < self.occupancy_grid.shape[0] and
                0 <= neighbor[1] < self.occupancy_grid.shape[1] and
                0 <= neighbor[2] < self.occupancy_grid.shape[2]):
                neighbors.append(neighbor)
        return neighbors

    def cost(self, a, b):
        """ a에서 b로 이동하는 비용 (장애물일 경우 INF) """
        if self.occupancy_grid[b] == 1:
            return INF
        return self.resolution

    def heuristic(self, a, b):
        (i1, j1, k1) = a
        (i2, j2, k2) = b
        return ((i1 - i2)**2 + (j1 - j2)**2 + (k1 - k2)**2) ** 0.5

    def reconstruct_path(self):
        current = self.start_idx
        path_idx = [current]
        max_steps = 1000
        steps = 0

        while current != self.goal_idx and steps < max_steps:
            steps += 1
            min_cost = INF
            next_node = None
            for s in self.get_neighbors(current):
                c = self.cost(current, s)
                total = c + self.g.get(s, INF)
                if total < min_cost:
                    min_cost = total
                    next_node = s

            if next_node is None or min_cost == INF:
                self.get_logger().error("🚨 경로 재구성 실패: 다음 노드를 찾을 수 없습니다. 경로 탐색 중단!")
                return None  # 강제 종료

            path_idx.append(next_node)
            current = next_node

        if steps >= max_steps:
            self.get_logger().error("🚨 경로 재구성 중 무한 루프 발생! 탐색 종료.")
            return None

        path_world = [self.grid_to_world(idx) for idx in path_idx]
        return path_world

    def run_dstar_lite(self):
        self.start_idx = self.world_to_grid(self.start_pos_world)
        self.goal_idx = self.world_to_grid(self.goal_pos_world)

        if self.occupancy_grid[self.start_idx] == 1:
            self.get_logger().error("🚨 출발 지점이 장애물 내부에 있습니다. 경로 탐색 불가!")
            return
        if self.occupancy_grid[self.goal_idx] == 1:
            self.get_logger().error("🚨 목표 지점이 장애물 내부에 있습니다. 경로 탐색 불가!")
            return

        self.get_logger().info("📌 D* Lite 경로 탐색 시작")
        self.initialize_dstar()
        self.compute_shortest_path()
        path = self.reconstruct_path()
        if path:
            self.publish_path(path)
        else:
            self.get_logger().warning("⚠️ 경로를 찾지 못했습니다.")

    def publish_path(self, path):
        """ 월드 좌표 경로를 ROS2 Path 메시지로 퍼블리시 """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            
            point_msg = Point()
            point_msg.x = float(point[0])
            point_msg.y = float(point[1])
            point_msg.z = float(point[2])
            
            pose.pose.position = point_msg
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        # self.get_logger().info(f"🛤️ 퍼블리시된 경로: {path}")

def main(args=None):
    rclpy.init(args=args)

    # 명령행 인자 처리 (첫 번째 인자가 있으면 사용, 없으면 기본값 "zed")
    parser_mode = sys.argv[1] if len(sys.argv) > 1 else "zed"

    # 올바른 값인지 검증 (zed 또는 custom만 허용)
    if parser_mode not in ["zed", "custom"]:
        print("❌ 잘못된 인자! 사용법: ros2 run path_generation dstar_lite_path_planner [NONE/custom]")
        return

    # 인자를 전달하여 노드 실행
    node = DStarLitePathPlanner(parser_mode)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
