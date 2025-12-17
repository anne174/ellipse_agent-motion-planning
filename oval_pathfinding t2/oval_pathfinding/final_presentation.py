import pygame
import sys

from implementation import diagram_20 
from visualize_anim import a_star_search_generator, draw_grid
from ellipse_agent import EllipseAgent

CELL_SIZE = 50
WINDOW_WIDTH = diagram_20.width * CELL_SIZE
WINDOW_HEIGHT = diagram_20.height * CELL_SIZE

def main():
    pygame.init()
    pygame.font.init()
    font = pygame.font.SysFont('Arial', 18)
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("自由收缩伸展的椭圆运动规划")
    clock = pygame.time.Clock()
    
    # 初始化
    start, goal = (3, 1), (12, 16)
    search_generator = a_star_search_generator(diagram_20, start, goal)
    
    # 状态变量
    came_from = {}
    cost_so_far = {}
    display_path = []
    current_node = start
    is_searching = True
    
    # 运动变量
    agent = None
    path_index = 0
    progress = 0.0
    speed = 0.05
    final_path = []

    background_surface = None

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        # --- 阶段一：搜索 ---
        if is_searching:
            clock.tick(30)
            try:
                # 复用搜索逻辑
                current_node, came_from, cost_so_far, found, temp_path = next(search_generator)
                display_path = temp_path
                draw_grid(screen, diagram_20, came_from, current_node, cost_so_far, font, display_path)
                if found:
                    is_searching = False
                    final_path = temp_path
                    # 初始化智能体
                    sx = start[0] * CELL_SIZE + CELL_SIZE // 2
                    sy = start[1] * CELL_SIZE + CELL_SIZE // 2
                    agent = EllipseAgent(sx, sy, area_constant=1500)
                    background_surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
                    draw_grid(background_surface, diagram_20, came_from, current_node, cost_so_far, font, display_path)
            except StopIteration:
                is_searching = False

        # --- 阶段二：运动 ---
        else:
            clock.tick(30)
           # 1. 智能控制逻辑
            if agent and path_index < len(final_path) - 1:
                curr_node = final_path[path_index]
                next_node = final_path[path_index + 1]
                
                # 根据障碍物自动调整姿态
                #agent.handle_keyboard()
                agent.auto_adjust(diagram_20, curr_node, next_node)
                
                # 沿路径移动
                path_index, progress = agent.move_along_path(final_path, path_index, progress, speed, CELL_SIZE)
            

        # --- 绘图 ---
        # 复用背景绘图
        if background_surface:
                screen.blit(background_surface, (0, 0))
        
        # 叠加椭圆
        if agent: 
            agent.draw(screen)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()