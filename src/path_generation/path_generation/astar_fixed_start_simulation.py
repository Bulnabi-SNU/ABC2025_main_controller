#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from nav_msgs.msg import Path  # 더 이상 사용하지 않지만, 기존 메시지 참고용
import numpy as np
import heapq
import sensor_msgs_py.point_cloud2 as pc2
import sys

INF = float('inf')

class AStarPathPlanner(Node):
    def __init__(self, parser_mode):
        super().__init__('astar_path_planner')

        if parser_mode == "custom":
            pointcloud_topic = "/custom_fused_cloud"
        else:
            pointcloud_topic = "/zed/zed_node/mapping/fused_cloud"

        self.create_subscription(PointCloud2, pointcloud_topic, self.pcl_callback, 10)
        self.create_subscription(Float32MultiArray, '/yolo_detection', self.balloon_callback, 10)
        # 기존 Path 메시지 대신, PoseStamped 메시지를 통해 한 웨이포인트씩 발행합니다.
        self.path_pub = self.create_publisher(PoseStamped, '/planned_waypoint', 10)
        self.time_period = 0.5  # 웨이포인트 발행 주기 (초)

        self.start_point_pub = self.create_publisher(PointStamped, '/start_point', 10)
        self.create_timer(self.time_period, self.publish_start_point)

        self.occupancy_grid = None
        self.grid_origin = None
        self.resolution = None

        self.start_pos_world = [0, 0, 0.5]
        self.goal_pos_world = None

        self.floor_z = None
        self.ceiling_z = None

        self.get_logger().info("🚀 AStarPathPlanner 실행됨")

    def find_floor_ceiling(self, points):
        z_coords = points[:, 2]
        hist, bin_edges = np.histogram(z_coords, bins=50)
        smoothed_hist = np.convolve(hist, np.ones(3)/3, mode='valid')
        peaks = []
        for i in range(1, len(smoothed_hist)-1):
            if smoothed_hist[i] > smoothed_hist[i-1] and smoothed_hist[i] > smoothed_hist[i+1]:
                peaks.append((smoothed_hist[i], bin_edges[i]))
        peaks.sort(reverse=True)
        if len(peaks) >= 2:
            z1, z2 = peaks[0][1], peaks[1][1]
            self.floor_z = min(z1, z2)
            self.ceiling_z = max(z1, z2)
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
        self.find_floor_ceiling(points)
        desired_resolution = 0.1
        self.occupancy_grid, self.grid_origin, self.resolution = self.create_occupancy_grid(points, desired_resolution)
        self.get_logger().info(f"📊 Occupancy Grid 생성 완료, shape: {self.occupancy_grid.shape}")
        if self.goal_pos_world:
            self.run_astar()

    def create_occupancy_grid(self, points, resolution):
        min_coords = np.min(points, axis=0) - resolution
        max_coords = np.max(points, axis=0) + resolution
        grid_shape = np.ceil((max_coords - min_coords) / resolution).astype(int) + 1
        grid = np.zeros(tuple(grid_shape), dtype=np.uint8)
        obstacle_radius = max(1, int(0.1 / resolution))
        indices = ((points - min_coords) / resolution).astype(int)
        valid_mask = np.all((indices >= 0) & (indices < grid_shape), axis=1)
        valid_indices = indices[valid_mask]
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
        directions = [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]
        neighbors = []
        for di, dj, dk in directions:
            neighbor = (i+di, j+dj, k+dk)
            new_world_z = self.grid_origin[2] + ((k + dk) * self.resolution)
            if (0 <= neighbor[0] < self.occupancy_grid.shape[0] and
                0 <= neighbor[1] < self.occupancy_grid.shape[1] and
                0 <= neighbor[2] < self.occupancy_grid.shape[2] and
                self.floor_z <= new_world_z <= self.ceiling_z and
                self.occupancy_grid[neighbor] == 0):
                neighbors.append(neighbor)
        return neighbors

    def heuristic(self, a, b):
        return np.sqrt(sum((np.array(a) - np.array(b)) ** 2))

    def astar(self, start, goal):
        frontier = [(0, start)]
        self.came_from = {start: None}
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

            # 기존 부분 경로 발행은 제거하여, 한 번에 최종 경로만 발행합니다.
            # if iterations % 100 == 0:
            #     partial_ = self.reconstruct_path(self.came_from, current)
            #     if partial_path:
            #         self.publish_path(partial_path, is_final=False)

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
        return path[::-1]

    def bezier_curve(self, points, num_points=50):
        def pascal_triangle(n):
            line = [1]
            for k in range(n):
                line.append(int(line[k] * (n - k) / (k + 1)))
            return line

        def bernstein_poly(i, n, t):
            coef = pascal_triangle(n)[i]
            return coef * (t ** i) * ((1 - t) ** (n - i))

        points = np.array(points, dtype=np.float64)
        n = len(points) - 1
        t = np.linspace(0, 1, num_points)
        curve = np.zeros((num_points, 3), dtype=np.float64)
        for i in range(n + 1):
            b = bernstein_poly(i, n, t).reshape(-1, 1)
            curve += np.tile(points[i], (num_points, 1)) * b
        return curve

    def smooth_path(self, path, max_control_points=10):
        if len(path) < 2:
            return path
        control_points = min(4, len(path))
        while control_points <= max_control_points:
            indices = np.linspace(0, len(path) - 1, control_points, dtype=int)
            control_path = [path[i] for i in indices]
            try:
                smoothed_points = self.bezier_curve(control_path)
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
                control_points += 2
            except Exception as e:
                self.get_logger().warning(f"⚠️ 스무딩 중 오류 발생: {str(e)}")
                break
        self.get_logger().warning("⚠️ 안전한 스무딩 경로를 찾지 못해 원본 경로를 사용합니다.")
        return path

    def start_publishing_path(self, path):
        # path는 [x, y, z] 좌표의 리스트입니다.
        self.planned_path = path
        self.path_index = 0
        self.get_logger().info("📤 최종 경로의 웨이포인트 발행 시작")
        self.path_timer = self.create_timer(self.time_period, self.publish_next_waypoint)

    def publish_next_waypoint(self):
        if self.path_index < len(self.planned_path):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            x, y, z = self.planned_path[self.path_index]
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            self.path_pub.publish(pose)
            self.get_logger().info(f"🚀 웨이포인트 {self.path_index} 발행: ({x}, {y}, {z})")
            self.path_index += 1
        else:
            self.get_logger().info("✅ 모든 웨이포인트 발행 완료")
            self.path_timer.cancel()

    def run_astar(self):
        start_idx = self.world_to_grid(self.start_pos_world)
        goal_idx = self.world_to_grid(self.goal_pos_world)

        if self.occupancy_grid[start_idx] == 1:
            self.get_logger().error("🚨 출발 지점이 장애물 내부에 있습니다!")
            return
        if self.occupancy_grid[goal_idx] == 1:
            self.get_logger().error("🚨 목표 지점이 장애물 내부에 있습니다!")
            return

        self.get_logger().info("📌 A* 경로 탐색 시작")
        path = self.astar(start_idx, goal_idx)
        if path:
            try:
                final_path = self.smooth_path(path)
                self.get_logger().info("✅ Final smoothed path computed")
            except Exception as e:
                self.get_logger().error(f"🚨 경로 스무딩 중 오류 발생: {str(e)}. 원본 경로 사용.")
                final_path = path
            self.start_publishing_path(final_path)
        else:
            self.get_logger().warning("⚠️ 경로를 찾지 못했습니다.")

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
