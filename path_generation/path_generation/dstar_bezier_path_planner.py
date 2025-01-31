import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import pcl  # PCL Point Cloud Library (pclpy 사용)

class DStarLitePathPlanner(Node):
    def __init__(self):
        super().__init__('dstar_lite_path_planner')

        # 구독자: ZED 포인트 클라우드 & 풍선 위치
        self.create_subscription(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', self.pcl_callback, 10)
        self.create_subscription(Float32MultiArray, '/yolo_detection', self.balloon_callback, 10)

        # 퍼블리셔: 생성된 경로
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.grid_size = 0.1  # 10cm 단위로 그리드 생성
        self.start_pos = [0, 0, 0]  # 드론 초기 위치
        self.goal_pos = None  # 풍선 위치
        self.obstacles = set()  # 장애물 좌표 저장

    def pcl_callback(self, msg):
        """ 실시간 장애물 정보 업데이트 """
        cloud = pcl.PointCloud.PointXYZ()
        # PCL 포인트 클라우드 변환 (PointCloud2에서 변환 필요)
        # 실제로 변환하려면 sensor_msgs.point_cloud2.read_points 사용 필요
        self.obstacles.clear()
        for i in range(100):  # 임의의 100개 장애물 추가
            x, y, z = np.random.uniform(-5, 5, size=3)
            self.obstacles.add((round(x, 1), round(y, 1), round(z, 1)))

        if self.goal_pos:
            self.run_dstar_lite()  # 장애물 변경 시 경로 재계산

    def balloon_callback(self, msg):
        """ 풍선 위치를 목표 지점으로 설정하고 D* Lite 실행 """
        self.goal_pos = [round(msg.data[0], 1), round(msg.data[1], 1), round(msg.data[2], 1)]
        self.run_dstar_lite()

    def run_dstar_lite(self):
        """ D* Lite 알고리즘 실행하여 경로 생성 """
        if self.goal_pos is None:
            return

        open_list = []
        heapq.heappush(open_list, (0, self.start_pos))
        came_from = {}
        g_score = {tuple(self.start_pos): 0}
        f_score = {tuple(self.start_pos): self.heuristic(self.start_pos, self.goal_pos)}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == self.goal_pos:
                raw_path = self.reconstruct_path(came_from, current)
                self.publish_path(raw_path)
                return

            for neighbor in self.get_neighbors(current):
                if tuple(neighbor) in self.obstacles:
                    continue  # 장애물 회피

                temp_g_score = g_score.get(tuple(current), float('inf')) + self.grid_size
                if temp_g_score < g_score.get(tuple(neighbor), float('inf')):
                    came_from[tuple(neighbor)] = current
                    g_score[tuple(neighbor)] = temp_g_score
                    f_score[tuple(neighbor)] = temp_g_score + self.heuristic(neighbor, self.goal_pos)
                    heapq.heappush(open_list, (f_score[tuple(neighbor)], neighbor))

    def heuristic(self, pos, goal):
        """ 휴리스틱 (맨해튼 거리) """
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1]) + abs(pos[2] - goal[2])

    def get_neighbors(self, pos):
        """ 인접 노드 (6방향 이동) """
        directions = [
            (self.grid_size, 0, 0), (-self.grid_size, 0, 0),
            (0, self.grid_size, 0), (0, -self.grid_size, 0),
            (0, 0, self.grid_size), (0, 0, -self.grid_size)
        ]
        return [[pos[0] + dx, pos[1] + dy, pos[2] + dz] for dx, dy, dz in directions]

    def reconstruct_path(self, came_from, current):
        """ 최적 경로를 생성 """
        path = []
        while tuple(current) in came_from:
            path.insert(0, current)
            current = came_from[tuple(current)]
        return path

    def publish_path(self, path):
        """ 경로를 ROS2 Path 메시지로 퍼블리시 """
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
