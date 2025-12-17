import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import random
import math
from collections import defaultdict, deque


class VisibilityPRM:
    def __init__(self, workspace_size=(100, 100), obstacles=None):
        """
        初始化基于可见性的PRM规划器

        参数:
        workspace_size: 工作空间尺寸 (width, height)
        obstacles: 障碍物列表，每个障碍物为 (x, y, width, height)
        """
        self.workspace_size = workspace_size
        self.obstacles = obstacles if obstacles else []

        # 算法参数
        self.M = 50  # 终止条件参数
        self.local_planner_max_steps = 20  # 局部规划器最大步数

        # 算法数据结构
        self.guards = []  # 守卫节点列表
        self.connections = []  # 连接节点列表
        self.edges = []  # 边列表
        self.components = []  # 连通分量列表，每个分量是一个守卫集合

        # 统计信息
        self.local_planner_calls = 0
        self.nodes_generated = 0

    def is_collision_free(self, q):
        """检查配置q是否无碰撞"""
        x, y = q
        for ox, oy, ow, oh in self.obstacles:
            if ox <= x <= ox + ow and oy <= y <= oy + oh:
                return False
        return 0 <= x <= self.workspace_size[0] and 0 <= y <= self.workspace_size[1]

    def local_planner(self, q1, q2):
        """
        局部规划器：检查从q1到q2的直线路径是否无碰撞

        返回:
        True: 路径无碰撞
        False: 路径有碰撞
        """
        self.local_planner_calls += 1

        # 简单直线插值检查
        steps = max(2, int(np.linalg.norm(np.array(q2) - np.array(q1)) / 2))
        for i in range(steps + 1):
            t = i / steps
            q = (q1[0] + t * (q2[0] - q1[0]),
                 q1[1] + t * (q2[1] - q1[1]))
            if not self.is_collision_free(q):
                return False
        return True

    def is_visible(self, guard, q):
        """检查配置q是否在guard的可见域内"""
        return self.local_planner(guard, q)

    def random_configuration(self):
        """生成随机无碰撞配置"""
        while True:
            x = random.uniform(0, self.workspace_size[0])
            y = random.uniform(0, self.workspace_size[1])
            q = (x, y)
            if self.is_collision_free(q):
                self.nodes_generated += 1
                return q

    def find_component(self, guard):
        """找到守卫所属的连通分量"""
        for i, component in enumerate(self.components):
            if guard in component:
                return i
        return -1

    def merge_components(self, comp_idx1, comp_idx2):
        """合并两个连通分量"""
        if comp_idx1 == comp_idx2:
            return comp_idx1

        # 确保comp_idx1是较小的索引
        if comp_idx1 > comp_idx2:
            comp_idx1, comp_idx2 = comp_idx2, comp_idx1

        # 合并分量
        self.components[comp_idx1].update(self.components[comp_idx2])
        del self.components[comp_idx2]

        return comp_idx1

    def build_roadmap(self, max_iterations=1000):
        """
        构建基于可见性的概率路图

        参数:
        max_iterations: 最大迭代次数

        返回:
        roadmap: 构建的路图信息
        """
        ntry = 0

        for iteration in range(max_iterations):
            if ntry >= self.M:
                print(f"算法终止: ntry={ntry} >= M={self.M}")
                break

            q = self.random_configuration()
            found_guard = None
            found_component_idx = -1

            # 检查所有连通分量
            for comp_idx, component in enumerate(self.components):
                found_in_component = False

                # 检查分量中的每个守卫
                for guard in component:
                    if self.is_visible(guard, q):
                        found_in_component = True

                        if found_guard is None:
                            # 第一次找到可见的守卫
                            found_guard = guard
                            found_component_idx = comp_idx
                        else:
                            # 找到第二个可见的守卫 -> q是连接点
                            # 添加连接点和边
                            self.connections.append(q)
                            self.edges.append((found_guard, q))
                            self.edges.append((guard, q))

                            # 合并连通分量
                            self.merge_components(found_component_idx, comp_idx)

                            # 跳出所有循环
                            found_guard = "connection"  # 特殊标记
                            break

                        # 论文算法: 在一个分量中找到第一个可见守卫后就跳到下一个分量
                        break

                if found_guard == "connection":
                    break

            # 根据检查结果处理
            if found_guard == "connection":
                # q已作为连接点添加，ntry不变
                pass
            elif found_guard is None:
                # q不被任何守卫看到 -> 成为新守卫
                self.guards.append(q)
                self.components.append({q})
                ntry = 0  # 重置失败计数器
                print(f"添加新守卫: {q}, 总守卫数: {len(self.guards)}")
            else:
                # q只被一个分量看到 -> 拒绝
                ntry += 1

            if iteration % 100 == 0:
                print(f"迭代 {iteration}: 守卫={len(self.guards)}, 连接点={len(self.connections)}, ntry={ntry}")

        print(f"路图构建完成: {len(self.guards)} 个守卫, {len(self.connections)} 个连接点, {len(self.edges)} 条边")
        print(f"局部规划器调用次数: {self.local_planner_calls}")
        print(f"生成的节点数: {self.nodes_generated}")

        return {
            'guards': self.guards,
            'connections': self.connections,
            'edges': self.edges,
            'components': self.components
        }

    def query(self, q_start, q_goal):
        """
        查询路径：连接起点和终点到路图，并寻找路径

        返回:
        path: 从起点到终点的路径，如果找不到则返回None
        """
        # 将起点和终点连接到路图
        start_guard = None
        goal_guard = None

        # 找到能看到起点和终点的守卫
        for guard in self.guards:
            if start_guard is None and self.is_visible(guard, q_start):
                start_guard = guard
            if goal_guard is None and self.is_visible(guard, q_goal):
                goal_guard = guard
            if start_guard and goal_guard:
                break

        if not start_guard or not goal_guard:
            print("无法将起点或终点连接到路图")
            return None

        # 构建图用于搜索
        graph = defaultdict(list)
        for edge in self.edges:
            graph[edge[0]].append(edge[1])
            graph[edge[1]].append(edge[0])

        # 添加起点和终点的连接
        graph[q_start].append(start_guard)
        graph[start_guard].append(q_start)
        graph[q_goal].append(goal_guard)
        graph[goal_guard].append(q_goal)

        # BFS搜索路径
        visited = set()
        queue = deque([(q_start, [q_start])])

        while queue:
            current, path = queue.popleft()

            if current == q_goal:
                return path

            if current in visited:
                continue
            visited.add(current)

            for neighbor in graph[current]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

        return None

    def visualize(self, path=None, q_start=None, q_goal=None):
        """可视化路图和路径"""
        plt.figure(figsize=(12, 10))

        # 绘制障碍物
        for ox, oy, ow, oh in self.obstacles:
            plt.gca().add_patch(Rectangle((ox, oy), ow, oh, fill=True, color='gray', alpha=0.7))

        # 绘制工作空间边界
        plt.gca().add_patch(Rectangle((0, 0), self.workspace_size[0], self.workspace_size[1],
                                      fill=False, color='black', linewidth=2))

        # 绘制守卫节点（红色）
        if self.guards:
            guards_x, guards_y = zip(*self.guards)
            plt.scatter(guards_x, guards_y, c='red', s=50, marker='o', label='Guards', zorder=3)

        # 绘制连接节点（蓝色）
        if self.connections:
            conn_x, conn_y = zip(*self.connections)
            plt.scatter(conn_x, conn_y, c='blue', s=30, marker='s', label='Connections', zorder=2)

        # 绘制边
        for edge in self.edges:
            x_vals = [edge[0][0], edge[1][0]]
            y_vals = [edge[0][1], edge[1][1]]
            plt.plot(x_vals, y_vals, 'g-', alpha=0.6, linewidth=1, zorder=1)

        # 绘制路径
        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_x, path_y, 'r-', linewidth=3, label='Path', zorder=4)

            # 标记起点和终点
            plt.scatter(path_x[0], path_y[0], c='green', s=100, marker='*', label='Start', zorder=5)
            plt.scatter(path_x[-1], path_y[-1], c='purple', s=100, marker='*', label='Goal', zorder=5)
        elif q_start and q_goal:
            # 只标记起点和终点
            plt.scatter(q_start[0], q_start[1], c='green', s=100, marker='*', label='Start', zorder=5)
            plt.scatter(q_goal[0], q_goal[1], c='purple', s=100, marker='*', label='Goal', zorder=5)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Visibility-based Probabilistic Roadmap')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.show()


# 示例使用
def main():
    # 创建工作空间和障碍物（模拟狭窄通道）
    workspace_size = (100, 100)
    obstacles = [
        (20, 0, 10, 40),  # 左下障碍物
        (20, 60, 10, 40),  # 左上障碍物
        (70, 0, 10, 40),  # 右下障碍物
        (70, 60, 10, 40),  # 右上障碍物
        (40, 45, 20, 10),  # 中间障碍物，形成狭窄通道
    ]

    # 创建规划器
    planner = VisibilityPRM(workspace_size, obstacles)

    # 构建路图
    print("开始构建基于可见性的概率路图...")
    roadmap = planner.build_roadmap(max_iterations=800)

    # 定义起点和终点
    start = (10, 10)
    goal = (90, 90)

    # 查询路径
    print(f"\n查询从 {start} 到 {goal} 的路径...")
    path = planner.query(start, goal)

    if path:
        print(f"找到路径! 路径长度: {len(path)}")
        print(f"路径: {path[:3]}...{path[-3:] if len(path) > 6 else ''}")
    else:
        print("未找到路径!")

    # 可视化结果
    planner.visualize(path, start, goal)

    # 打印算法统计
    print(f"\n算法统计:")
    print(f"- 守卫节点数: {len(planner.guards)}")
    print(f"- 连接节点数: {len(planner.connections)}")
    print(f"- 边数: {len(planner.edges)}")
    print(f"- 连通分量数: {len(planner.components)}")
    print(f"- 局部规划器调用次数: {planner.local_planner_calls}")
    print(f"- 生成的节点数: {planner.nodes_generated}")


if __name__ == "__main__":
    main()