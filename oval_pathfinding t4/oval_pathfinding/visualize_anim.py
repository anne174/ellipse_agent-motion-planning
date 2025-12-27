import pygame
#import sys
#import heapq # 需要重新导入这个以重写搜索逻辑用于演示
from implementation import diagram_20, GridLocation, PriorityQueue, heuristic, reconstruct_path,ORIGINAL_WALLS

# --- 动画版 A* 搜索函数 ---
# 我们把原本一次性跑完的函数，改成一个生成器
def a_star_search_generator(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()

        current_path = []
        if current in came_from:
            current_path = reconstruct_path(came_from, start, current)
        # 每次取出一个节点，就“暂停”一下，返回当前状态给绘图函数
        yield current, came_from, cost_so_far, False, current_path # False表示还没找到终点
        
        if current == goal:
            final_path = reconstruct_path(came_from, start, goal)
            yield current, came_from, cost_so_far, True, final_path # True表示找到了
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current

# --- 绘图逻辑 ---
CELL_SIZE = 20
MARGIN = 2
colors = {
    'wall': (20, 20, 20),# 实心墙 (黑色)
    'inflated':(169,169,169),# 【新增】膨胀区
    'path': (0, 0, 255),
    'visited': (100, 200, 255),
    'current': (255, 255, 0),
    'empty': (255, 255, 255),
    'start': (0, 255, 0),
    'goal': (255, 0, 0)
}

def draw_grid(screen, graph, came_from, current_node, cost_so_far, font, path=[]):
    screen.fill((0, 0, 0))

    for y in range(graph.height):
        for x in range(graph.width):
            node = (x, y)
            rect = pygame.Rect (x * CELL_SIZE + MARGIN, y * CELL_SIZE + MARGIN, 
                    CELL_SIZE - 2*MARGIN, CELL_SIZE - 2*MARGIN)
            
            if node in ORIGINAL_WALLS: color = colors['wall']
            elif node in graph.walls: color=colors['inflated'] # 膨胀区域显示
            elif node == (3, 1): color = colors['start'] # 硬编码起点用于演示
            elif node == (25, 25): color = colors['goal']  # 硬编码终点用于演示
            elif node in path: color = colors['path']
            elif node == current_node: color = colors['current'] # 高亮当前头节点
            elif node in came_from: color = colors['visited']
            else: color = colors['empty']
            
            pygame.draw.rect(screen, color, rect)

            if node in cost_so_far:
                cost_value = int(cost_so_far[node]) #以此处代价为例，取整显示更简洁
                # 渲染文字：内容, 抗锯齿, 颜色(黑色)
                text_surf = font.render(str(cost_value), True, (0, 0, 0))
                # 获取文字的矩形区域，并将其中心对齐到格子中心
                text_rect = text_surf.get_rect(center=rect.center)
                screen.blit(text_surf, text_rect)
    pygame.display.flip()

def main():
    pygame.init()
    pygame.font.init()
    font = pygame.font.SysFont('Arial', 18)

    #from implementation import diagram_20, reconstruct_path

    screen = pygame.display.set_mode((diagram_20.width * CELL_SIZE, diagram_20.height * CELL_SIZE))
    pygame.display.set_caption("A* 算法 可视化")
    clock = pygame.time.Clock()
    
    start, goal = (3, 1), (25, 25)
    
    # 初始化生成器
    search_generator = a_star_search_generator(diagram_20, start, goal)
    
    display_path = []

    running = True
    finished = False
    
    came_from = {}
    cost_so_far = {}
    current = start

    while running:
        
        clock.tick(5) 
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        if not finished:
            try:
                # 获取算法的下一步状态
                current, came_from, cost_so_far, is_goal, path_so_far = next(search_generator)
                display_path = path_so_far
                if is_goal:
                    finished = True
                
            except StopIteration:
                finished = True
        
        draw_grid(screen, diagram_20, came_from, current, cost_so_far, font, display_path)

    pygame.quit()

if __name__ == "__main__":
    main()