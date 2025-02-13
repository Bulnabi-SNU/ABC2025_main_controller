import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3D plotting
import heapq
import random
import math

# --------------------------
# 3차원 A* 알고리즘에 사용할 노드 클래스
# --------------------------
class Node:
    def __init__(self, pos, g=float('inf'), f=float('inf')):
        self.pos = pos  # (x, y, z)
        self.g = g      # 시작점부터 해당 노드까지의 실제 비용
        self.f = f      # f = g + h (휴리스틱)
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f

# --------------------------
# 휴리스틱 함수 (유클리드 거리 사용)
# --------------------------
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

# --------------------------
# 3차원 A* 알고리즘 (최종 경로만 구함)
# --------------------------
def astar_3d(grid, start, goal):
    x_dim, y_dim, z_dim = grid.shape
    nodes = {}
    for x in range(x_dim):
        for y in range(y_dim):
            for z in range(z_dim):
                nodes[(x, y, z)] = Node((x, y, z))
    # 시작점 초기화
    nodes[start].g = 0
    nodes[start].f = heuristic(start, goal)

    open_set = []
    heapq.heappush(open_set, (nodes[start].f, start))
    open_set_hash = {start}
    closed_set = set()

    # 알고리즘 실행 (중간 과정은 생략)
    while open_set:
        current_f, current_pos = heapq.heappop(open_set)
        open_set_hash.discard(current_pos)
        current_node = nodes[current_pos]
        closed_set.add(current_pos)

        if current_pos == goal:
            break

        # 인접 노드 (6방향: ±x, ±y, ±z)
        for dx, dy, dz in [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]:
            nx, ny, nz = current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz
            if 0 <= nx < x_dim and 0 <= ny < y_dim and 0 <= nz < z_dim:
                if grid[nx, ny, nz] == 1:
                    continue  # 장애물은 건너뜀
                neighbor_pos = (nx, ny, nz)
                if neighbor_pos in closed_set:
                    continue
                tentative_g = current_node.g + 1  # 인접 이동 비용 1
                neighbor_node = nodes[neighbor_pos]
                if tentative_g < neighbor_node.g:
                    neighbor_node.parent = current_node
                    neighbor_node.g = tentative_g
                    neighbor_node.f = tentative_g + heuristic(neighbor_pos, goal)
                    if neighbor_pos not in open_set_hash:
                        heapq.heappush(open_set, (neighbor_node.f, neighbor_pos))
                        open_set_hash.add(neighbor_pos)
    # 최종 경로 재구성
    path = []
    current = nodes[goal]
    if current.parent or current.pos == start:
        while current is not None:
            path.append(current.pos)
            current = current.parent
        path.reverse()
    return path

# --------------------------
# 베지에 커브 계산 함수
# --------------------------
def bezier_curve(control_points, n_points=200):
    """
    control_points: list of 제어점, 각 제어점은 (x, y, z) 튜플 또는 배열
    n_points: 커브상에서 생성할 포인트 개수
    반환: (n_points, 3) 모양의 numpy 배열
    """
    control_points = np.array(control_points)
    n = len(control_points) - 1  # 베지에 커브의 차수
    curve_points = []
    # t를 0부터 1까지 균일하게 분할
    for t in np.linspace(0, 1, n_points):
        point = np.zeros(3)
        for i, P in enumerate(control_points):
            # 이항 계수 (Python 3.8 이상에서는 math.comb 사용)
            binom = math.comb(n, i)
            point += binom * ((1 - t) ** (n - i)) * (t ** i) * P
        curve_points.append(point)
    return np.array(curve_points)

# --------------------------
# 최종 경로와 베지에 커브를 3D 플롯으로 시각화
# --------------------------
def draw_final_path_3d(grid, start, goal, path, smooth_path):
    x_dim, y_dim, z_dim = grid.shape
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D A* 최종 경로 & 부드러운 베지에 커브", fontsize=16)
    ax.set_xlim(0, x_dim-1)
    ax.set_ylim(0, y_dim-1)
    ax.set_zlim(0, z_dim-1)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # 장애물 표시 (검은색 큐브)
    obstacle_points = np.argwhere(grid == 1)
    if obstacle_points.size > 0:
        ax.scatter(obstacle_points[:, 2], obstacle_points[:, 1],
                   obstacle_points[:, 0], c='black', marker='s', s=100, label="Obstacle")

    # 시작점과 목표점 표시
    ax.scatter(start[2], start[1], start[0], c='green', marker='o', s=200, label="Start")
    ax.scatter(goal[2], goal[1], goal[0], c='red', marker='*', s=200, label="Goal")

    # 최종 경로 표시 (원래 A* 경로, 파란 선)
    if path:
        xs = [p[2] for p in path]
        ys = [p[1] for p in path]
        zs = [p[0] for p in path]
        ax.plot(xs, ys, zs, color='blue', linewidth=2, label="A* Path")

    # 베지에 커브 (부드러운 경로) 표시 (마젠타 선)
    if smooth_path is not None:
        xs = smooth_path[:, 2]
        ys = smooth_path[:, 1]
        zs = smooth_path[:, 0]
        ax.plot(xs, ys, zs, color='magenta', linewidth=3, label="Bezier Smoothed Path")

    ax.legend(loc='upper left')
    plt.show()

# --------------------------
# 메인 실행 코드
# --------------------------
if __name__ == "__main__":
    # 격자 크기 (11×11×11)
    x_dim, y_dim, z_dim = 11, 11, 11
    grid = np.zeros((x_dim, y_dim, z_dim), dtype=int)
    start = (0, 0, 0)
    goal = (10, 10, 10)

    # 장애물 생성 (20% 확률, 시작점과 목표점 제외)
    obstacle_probability = 0.2
    random.seed(42)
    for x in range(x_dim):
        for y in range(y_dim):
            for z in range(z_dim):
                if (x, y, z) == start or (x, y, z) == goal:
                    continue
                if random.random() < obstacle_probability:
                    grid[x, y, z] = 1

    # 3D A* 알고리즘 실행하여 최종 경로 계산
    path = astar_3d(grid, start, goal)
    print("A* 최종 경로:", path)

    # 베지에 커브로 부드럽게 보간 (제어점으로 A* 경로 사용)
    if path and len(path) >= 2:
        smooth_path = bezier_curve(path, n_points=300)
    else:
        smooth_path = None

    # 최종 경로와 부드러운 베지에 커브를 3D 플롯으로 표시
    draw_final_path_3d(grid, start, goal, path, smooth_path)
