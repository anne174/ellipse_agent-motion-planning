import pygame
import sys

from implementation import diagram_20, a_star_search, reconstruct_path

# --- 1. 配置参数 ---
CELL_SIZE = 50    # 每个格子在屏幕上的大小 (像素)
MARGIN = 2        # 格子之间的缝隙
# 根据地图尺寸自动计算窗口大小
WINDOW_WIDTH = diagram_20.width * CELL_SIZE
WINDOW_HEIGHT = diagram_20.height * CELL_SIZE

# --- 2. 颜色定义 (RGB) ---
WHITE = (255, 255, 255)      # 路 (空地)
BLACK = (20, 20, 20)         # 墙 (障碍物)
GRAY = (150, 150, 150)       # 泥潭 (高代价区域)
BLUE = (0, 120, 255)         # 最终路径
LIGHT_BLUE = (173, 216, 230) # 搜索过的区域 
GREEN = (0, 200, 0)          # 起点
RED = (200, 0, 0)            # 终点

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("A* 算法结果可视化")

    # --- 3. 运行算法 ---
    # 这里直接调用 implementation.py 里的逻辑
    start = (3, 1)
    goal = (12, 16)
    came_from, cost_so_far = a_star_search(diagram_20, start, goal)
    path = reconstruct_path(came_from, start, goal)

    # --- 4. 绘图循环 ---
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

        screen.fill(BLACK) # 背景色

        # 遍历地图的每个坐标 (x, y)
        for y in range(diagram_20.height):
            for x in range(diagram_20.width):
                node = (x, y)
                #节点在格子左上角
                rect = (x * CELL_SIZE + MARGIN, y * CELL_SIZE + MARGIN, 
                        CELL_SIZE - MARGIN, CELL_SIZE - MARGIN)

                # 判定颜色优先级
                if node == start:
                    color = GREEN
                elif node == goal:
                    color = RED
                elif node in path:      # 如果节点在最终路径列表中
                    color = BLUE
                elif node in diagram_20.walls: # 如果是墙
                    color = BLACK
                elif node in came_from: # 如果被算法“访问”过 (在came_from字典里)
                    color = LIGHT_BLUE
                elif node in diagram_20.weights: # 如果是高权重区域
                    color = GRAY
                else:
                    color = WHITE
                
                # 绘制矩形
                pygame.draw.rect(screen, color, rect)
        pygame.display.flip() # 刷新屏幕

if __name__ == "__main__":
    main()