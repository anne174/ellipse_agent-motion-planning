import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
import math
from matplotlib.patches import Ellipse

# 尝试使用 TkAgg，避免某些环境下的绘图报错
try:
    import matplotlib

    matplotlib.use("TkAgg")
except Exception:
    pass

# --------- 物理与几何参数 ----------
GRID_SIZE = 30  # 网格大小
DEFAULT_RADIUS = 1.5  # 默认半径（圆形状态）
# 面积常数因子 K = a * b. 对于半径为 R 的圆, a=R, b=R => K = R^2
# 实际面积 S = pi * K. 我们只维护 K 即可。
AREA_CONSTANT_K = DEFAULT_RADIUS * DEFAULT_RADIUS
MIN_AXIS_LEN = 0.4  # 物理限制：椭圆最扁不能小于这个半轴长


# --------- 辅助函数 ----------

def get_clearance(r, c, grid):
    """
    计算在点 (r, c) 处，水平和垂直方向到最近障碍物的距离（单位：网格格数）。
    返回: (max_width, max_height) 的可用空间
    """
    rows, cols = grid.shape

    # 向四个方向射线检测距离
    # Up (-x), Down (+x), Left (-y), Right (+y)
    d_up, d_down, d_left, d_right = 0, 0, 0, 0

    # 向上扫描
    for i in range(1, int(rows)):
        if r - i < 0 or grid[r - i, c] == 1: break
        d_up += 1
    # 向下扫描
    for i in range(1, int(rows)):
        if r + i >= rows or grid[r + i, c] == 1: break
        d_down += 1
    # 向左扫描
    for i in range(1, int(cols)):
        if c - i < 0 or grid[r, c - i] == 1: break
        d_left += 1
    # 向右扫描
    for i in range(1, int(cols)):
        if c + i >= cols or grid[r, c + i] == 1: break
        d_right += 1

    # 可用空间直径 = 两个方向的距离 + 当前中心点(视作1个单位宽?
    # 这里简化为：中心点到墙壁的距离 * 2，取最小的那边作为直径限制，或者直接用左右距离之和)
    # 为了保守避障，假设机器人中心在格子中心，可用宽度 = (左空闲 + 右空闲 + 1)
    # 但这可能导致中心贴墙。更严谨的是：max_radius = min(dist_to_wall)。
    # 这里采用：在该点能撑开的最大轴长 = 2 * min(单向距离) + 1 (本身占1格)
    # 为了让椭圆能贴墙走，我们计算中心到最近障碍物的距离

    # 垂直方向允许的最大半轴长 (Height / 2)
    limit_h = min(d_up, d_down) + 0.5  # +0.5 是假设障碍物在格子边缘
    # 水平方向允许的最大半轴长 (Width / 2)
    limit_w = min(d_left, d_right) + 0.5

    return limit_w, limit_h


def calculate_shape(limit_w, limit_h):
    """
    根据空间限制计算椭圆形状 (a, b)。
    约束1: a * b = AREA_CONSTANT_K
    约束2: a <= limit_w
    约束3: b <= limit_h
    """
    # 1. 尝试保持默认形状
    target_a = math.sqrt(AREA_CONSTANT_K)
    target_b = target_a

    # 2. 检查默认形状是否可行
    if target_a <= limit_w and target_b <= limit_h:
        return target_a, target_b, True  # 无需形变

    # 3. 需要形变：优先压缩受限更严重的轴
    # 假设我们需要压缩 a (水平宽度受限)
    # 设 a = limit_w，则必须有 b = K / limit_w。
    # 然后检查算出来的 b 是否 <= limit_h。

    # 尝试方案 A：卡住水平极限
    a_try1 = limit_w
    b_try1 = AREA_CONSTANT_K / a_try1 if a_try1 > 0 else float('inf')

    # 尝试方案 B：卡住垂直极限
    b_try2 = limit_h
    a_try2 = AREA_CONSTANT_K / b_try2 if b_try2 > 0 else float('inf')

    valid_shapes = []

    # 验证方案 A
    if a_try1 >= MIN_AXIS_LEN and b_try1 >= MIN_AXIS_LEN:  # 物理极限检查
        if b_try1 <= limit_h:  # 几何通过检查
            valid_shapes.append((a_try1, b_try1))

    # 验证方案 B
    if a_try2 >= MIN_AXIS_LEN and b_try2 >= MIN_AXIS_LEN:
        if a_try2 <= limit_w:
            valid_shapes.append((a_try2, b_try2))

    if not valid_shapes:
        return None, None, False  # 无法通过，堵死了

    # 如果有多个解，选择最接近圆形的（长宽比最接近1）或者变动最小的
    # 这里简单选择第一个可行解
    return valid_shapes[0][0], valid_shapes[0][1], True


# --------- A* 算法 ----------

def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search_deformable(start, goal, grid):
    rows, cols = grid.shape
    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]  # 增加对角线移动让动作更平滑

    # 检查起点和终点是否本身就是障碍
    if grid[start] == 1 or grid[goal] == 1:
        return None, 0, 0, 0

    open_heap = []
    heapq.heappush(open_heap, (0 + manhattan_distance(start, goal), 0, start))

    came_from = {}
    g = {start: 0}

    # 记录每个节点上的椭圆形状 (a, b)，用于后续绘图
    node_shapes = {}

    # 初始位置形状计算
    lw, lh = get_clearance(start[0], start[1], grid)
    sa, sb, valid = calculate_shape(lw, lh)
    if valid:
        node_shapes[start] = (sa, sb)
    else:
        print("警告：起点位置空间不足以放下机器人！")
        return None, 0, 0, 0

    expanded_count = 0
    frontier_max = 1
    t0 = time.time()

    while open_heap:
        f, g_curr, current = heapq.heappop(open_heap)
        expanded_count += 1

        if current == goal:
            # 回溯
            path = []
            node = current
            path_shapes = []  # 存储路径上的形状

            while node in came_from:
                path.append(node)
                path_shapes.append(node_shapes.get(node, (1, 1)))
                node = came_from[node]
            path.append(start)
            path_shapes.append(node_shapes[start])

            path.reverse()
            path_shapes.reverse()

            return path, path_shapes, expanded_count, time.time() - t0

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy

            # 1. 基础越界和障碍检查
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] == 0:

                # 2. 高级检查：形变可行性检查
                # 获取当前邻居节点的环境限制
                limit_w, limit_h = get_clearance(nx, ny, grid)
                # 计算是否能塞进去，以及形状是啥
                req_a, req_b, is_feasible = calculate_shape(limit_w, limit_h)

                if is_feasible:
                    move_cost = math.sqrt(dx ** 2 + dy ** 2)  # 直线1，斜线1.4SS14
                    tentative_g = g[current] + move_cost

                    if tentative_g < g.get((nx, ny), float('inf')):
                        came_from[(nx, ny)] = current
                        g[(nx, ny)] = tentative_g
                        f_val = tentative_g + manhattan_distance((nx, ny), goal)
                        heapq.heappush(open_heap, (f_val, tentative_g, (nx, ny)))

                        # 记录该节点的可行形状
                        node_shapes[(nx, ny)] = (req_a, req_b)

                        frontier_max = max(frontier_max, len(open_heap))

    return None, None, expanded_count, time.time() - t0


if __name__ == "__main__":
    # 1. 构建地图 (30x30) 以便展示细节
    size = 30
    grid = np.zeros((size, size), dtype=int)

    # 2. 设置障碍物 (构建狭窄通道)
    # 构建一堵墙，中间留个小缝隙
    for r in range(0, size):
        grid[r, 15] = 1  # 垂直墙

    # 在墙上开洞
    # 洞1：宽洞（不需要形变）
    grid[5, 15] = 0
    grid[6, 15] = 0
    grid[7, 15] = 0

    # 洞2：窄洞（需要压扁变宽才能过）
    # 假设正常直径是 3 (R=1.5)，这里只留 1 格宽
    grid[20, 15] = 0
    grid[20, 14] = 0  # 加宽入口让它好进一点
    grid[20, 16] = 0  # 加宽出口

    # 增加一些杂乱障碍
    grid[10:15, 5:10] = 1

    # 起点与终点
    start = (20, 5)  # 对应窄洞的入口侧
    goal = (20, 25)  # 对应窄洞的出口侧

    print("=== 可变形椭圆 A* 搜索 ===")
    print(f"机器人参数: 默认半径={DEFAULT_RADIUS}, 面积常数K={AREA_CONSTANT_K:.2f}")

    path, shapes, expanded, search_time = a_star_search_deformable(start, goal, grid)

    if path:
        print(f"搜索成功! 耗时: {search_time:.4f}s, 步数: {len(path)}")
    else:
        print("搜索失败：无解或无法通过狭窄区域。")

    # --------- 可视化 ---------
    fig, ax = plt.subplots(figsize=(8, 8))

    # 1. 绘制网格背景
    ax.imshow(grid, cmap="Greys", origin="upper")

    # 2. 绘制路径线
    if path:
        ys = [r for r, c in path]
        xs = [c for r, c in path]
        ax.plot(xs, ys, color="blue", linewidth=1, linestyle="--", alpha=0.5)

        # 3. 绘制可变形椭圆
        # 为了不画得太密，每隔几个点画一个，或者画全部
        step_skip = 1
        for i in range(0, len(path), step_skip):
            r, c = path[i]
            semi_a, semi_b = shapes[i]  # 半轴长

            # Matplotlib Ellipse 参数是 (x, y, width, height)
            # width = 2 * a (这里 a 对应 x 轴半轴? 取决于 calculate_shape 的逻辑)
            # 在 calculate_shape 中，第一个返回值对应 limit_w (x轴约束)，所以是 x轴半轴

            width = semi_a * 2
            height = semi_b * 2

            # 颜色根据形变程度变化：越圆越绿，越扁越红
            deformation_ratio = min(semi_a, semi_b) / max(semi_a, semi_b)
            color = (1 - deformation_ratio, deformation_ratio, 0)  # R G B

            el = Ellipse(xy=(c, r), width=width, height=height, angle=0,
                         edgecolor=color, facecolor=(*color, 0.3), linewidth=1.5)
            ax.add_patch(el)

            # 特别标记关键帧：如果形变严重
            if deformation_ratio < 0.8:
                pass

    # 标记起终点
    ax.scatter(start[1], start[0], c="blue", marker="o", label="Start")
    ax.scatter(goal[1], goal[0], c="green", marker="x", label="Goal")

    plt.title("Deformable Ellipse Pathfinding\n(Color: Green=Normal, Red=Squeezed)")
    plt.grid(False)  # 关闭默认网格线，因为 imshow 自带像素格
    plt.legend()
    plt.tight_layout()
    plt.show()
