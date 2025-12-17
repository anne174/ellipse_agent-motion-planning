import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import heapq
import time
import matplotlib
import math

# python数据结构
#列表：有序、可变（可增删改）、允许重复元素，用 [] 创建。
#元组：有序、不可变（创建后不能修改）、允许重复元素，用 () 创建
#字典：键值对（key: value）结构，键唯一且可哈希（如字符串、元组），有序，用 {} 创建
#集合：无序、无重复元素，用 {} 或 set() 创建（空集合必须用 set()，{} 是字典）
#队列：先进先出（FIFO），线程安全，通过 queue 模块实现。
#栈：后进先出（LIFO），可通过 list 模拟（简单场景）或 collections.deque（高效）实现
#堆：最小堆（默认），通过 heapq 模块实现，堆顶始终是最小元素
#链表：节点通过指针链接，动态扩容，需自定义类实现
       # 创建链表（头节点→节点1→节点2）
       #head = Node(1)
       #node2 = Node(2)
       #node3 = Node(3)
       #head.next = node2
       #node2.next = node3

# 使用 TkAgg 后端（如果支持）
try:
    matplotlib.use("TkAgg")
except Exception:
    pass


def check_collision_circle(grid, center_r, center_c, radius):
    """
    圆形碰撞检测
    检查以 (center_r, center_c) 为圆心，radius 为半径的范围内是否有障碍物
    """
    rows, cols = grid.shape

    # 1. 确定包围盒 (Bounding Box) 范围，减少计算量
    # 只检查圆周围的方形区域，不用遍历全图
    # ceil向上取整确保覆盖，max/min防止越界
    r_min = max(0, int(math.floor(center_r - radius)))
    r_max = min(rows, int(math.ceil(center_r + radius)))
    c_min = max(0, int(math.floor(center_c - radius)))
    c_max = min(cols, int(math.ceil(center_c + radius)))

    # 2. 遍历包围盒内的所有格子
    for r in range(r_min, r_max + 1):  # range是左闭右开，所以要+1
        for c in range(c_min, c_max + 1):
            # 3. 如果这个格子是障碍物
            if 0 <= r < rows and 0 <= c < cols and grid[r, c] == 1:
                # 4. 计算障碍物格子中心到机器人中心的距离
                # (这里简化处理，认为障碍物是点，严格来说应该算到障碍物边缘)
                dist = math.sqrt((r - center_r) ** 2 + (c - center_c) ** 2)

                # 5. 如果距离小于半径，说明撞上了
                if dist <= radius:
                    return True  # 发生碰撞

    return False  # 安全


# --------- A* 算法 ----------
def manhattan_distance(a, b):
    """曼哈顿距离作为启发式函数"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean_distance(a, b):
    """欧式距离"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def a_star_search(start, goal, grid,radius):
    """
    A* 搜索
    start: 起点 (r, c)
    goal: 终点 (r, c)
    grid: 迷宫numpy矩阵，0=可通行，1=障碍
    """
    rows, cols = grid.shape
    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]#定义邻居就定义了可以移动的方向（上下左右）如果想要定义对角线（1，1）（1，-1）（-1，1）（-1，-1）
    #（1，0）：行数+1——>向下移动    （-1，0）：向上移动    （0，1）：向右移动    （0，-1）：向左移动

    # 检查起点和终点是否本身就“违规”（离障碍物太近）
    if check_collision_circle(grid, start[0], start[1], radius):
        print("错误: 起点离障碍物太近！")
        return None, 0, 0, 0
    if check_collision_circle(grid, goal[0], goal[1], radius):
        print("错误: 终点离障碍物太近！")
        return None, 0, 0, 0

    open_heap = []#本质是一个优先队列，用于存储待探索的节点
    # 这里把起点以(f,g,node（当前节点位置）)元组的结构放入到一个优先队列（小根堆）中，这样f最小的节点总是在堆顶
    heapq.heappush(open_heap, (0 + manhattan_distance(start, goal), 0, start))#f:0+manhattan_distance;g:0(实际代价，因为还未开始移动所以为0）
    # 以字典的形式记录节点的父节点（子节点：父节点），方便回溯
    came_from = {}
    g = {start: 0}      # 存储每个节点到起点的实际代价
    expanded_count = 0  # 扩展节点数
    frontier_max = 1    # 存放待扩展节点的最大长度，非必须，可以用来佐证启发式函数的优劣

    t0 = time.time()
    while open_heap:#用open_heap作为while循环的条件表示只要open_heap里面有节点就会进行循环
        f, g_curr, current = heapq.heappop(open_heap)
        expanded_count += 1

        if current == goal:
            # 回溯路径
            path = []
            node = current
            while node in came_from:#python字典中默认搜索针对的是key而不是value
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()#原来的path的顺序是从目标节点一步步回溯到起始节点，而我们需要的是起始节点—>目标节点
            search_time = time.time() - t0
            return path, expanded_count, frontier_max, search_time

        # 遍历当前节点的四个可能移动方向（上下左右）
        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy  # 计算邻居节点的坐标

            # 判断邻居节点是否在地图范围内，并且不是障碍物
            if 0 <= nx < rows and 0 <= ny < cols :
                # 调用新的圆形碰撞检测
                if not check_collision_circle(grid, nx, ny, radius):

                   # tentative_g 是从起点到邻居节点（nx,ny）的代价（当前节点代价 + 1）
                   tentative_g = g[current] + 1

                   # 如果这是第一次访问这个邻居节点，或者找到了一条更短路径
                   if tentative_g < g.get((nx, ny), float('inf')):#如果是第一次访问，则g.get()会被赋予无穷大
                      # 更新 came_from，用于回溯路径：
                      # 表示到达 (nx, ny) 的最优前驱节点是 current
                      came_from[(nx, ny)] = current

                      # 更新从起点到邻居节点（nx,ny）的最小代价 g
                      g[(nx, ny)] = tentative_g

                      # 计算 f = g + h
                      # g 是从起点到邻居节点的代价，h 是邻居节点到目标的启发式估价
                      f_neighbor = tentative_g + manhattan_distance((nx, ny), goal)

                      # 将邻居节点加入 open_heap（优先队列），按 f 值排序
                      # 元组 (f, g, node) 中 f 用于排序，g 可以用作 tie-break
                      heapq.heappush(open_heap, (f_neighbor, tentative_g, (nx, ny)))

                      # 更新 frontier_max，用于统计 open_heap（frontier）最大长度
                      frontier_max = max(frontier_max, len(open_heap))

    search_time = time.time() - t0
    return None, expanded_count, frontier_max, search_time  # 无解



if __name__ == "__main__":
    size = 15
    grid = np.zeros((size, size), dtype=int)
    # 障碍物
    obstacles = []
    # 墙壁 1
    for i in range(0, 10): obstacles.append((i, 5))
    # 墙壁 2 (留出一个口子)
    for i in range(5, 15): obstacles.append((i, 9))
    # 随机散点
    obstacles += [(2, 2), (12, 2), (3, 12)]
    for r, c in obstacles:
        if 0 <= r < size and 0 <= c < size:
            grid[r, c] = 1
    # 起始点和目标点坐标定义
    start = (2, 0)
    goal = (size - 2, size - 1)

    # --- 设置机器人半径 ---
    # 尝试修改这个值：
    # 0.1 -> 像点一样，紧贴墙壁走
    # 1.1 -> 像个大圆，会离墙壁保持距离，甚至可能过不去窄口
    ROBOT_RADIUS =0.28
    print(f"=== A* 搜索开始 (机器人半径: {ROBOT_RADIUS}) ===")

    path, expanded, frontier_max, search_time = a_star_search(start, goal, grid, ROBOT_RADIUS)

    if path:
        print("是否有解: 是")
        print(f"搜索耗时: {search_time:.6f} 秒")
        print(f"路径长度: {len(path) - 1}")
    else:
        print("是否有解: 否 (可能是半径太大被卡住了)")

        # --------- 可视化 ---------
    fig, ax = plt.subplots(figsize=(8, 8))
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    plt.rcParams["axes.unicode_minus"] = False

    ax.imshow(grid, cmap="Greys", origin="upper")

    # 画起点和终点
    ax.scatter(start[1], start[0], c="red", marker="o", s=50, label="起点中心")
    ax.scatter(goal[1], goal[0], c="green", marker="X", s=50, label="终点中心")

    # 画路径
    if path:
        xs = [c for (r, c) in path]
        ys = [r for (r, c) in path]
        ax.plot(xs, ys, color="cyan", linewidth=2, marker=".", markersize=5, label="路径中心线")

        # 可视化路径上的“圆” (画出每一步的体积)
        # 为了不画太满，每隔几步画一个圆
        for i in range(0, len(path), 3):
            r, c = path[i]
            circle = Circle((c, r), ROBOT_RADIUS, color='blue', alpha=0.2)
            ax.add_patch(circle)

        # 单独画出起点和终点的圆范围
        start_circle = Circle((start[1], start[0]), ROBOT_RADIUS, color='red', alpha=0.3, label="起点范围")
        goal_circle = Circle((goal[1], goal[0]), ROBOT_RADIUS, color='green', alpha=0.3, label="终点范围")
        ax.add_patch(start_circle)
        ax.add_patch(goal_circle)

    ax.grid(which="both", color="black", linewidth=0.5)
    ax.set_xticks(np.arange(-0.5, size, 1))
    ax.set_yticks(np.arange(-0.5, size, 1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    ax.set_title(f"A* 圆形机器人路径 (半径={ROBOT_RADIUS})")
    ax.legend(loc='upper right')
    plt.show()
