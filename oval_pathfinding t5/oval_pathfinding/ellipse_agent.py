import pygame
import math

class EllipseAgent:
    def __init__(self, center_x, center_y, area_constant=850):
        # 1. 物理属性
        self.x = center_x
        self.y = center_y
        self.angle = 0  
            
        
        # 2. 几何属性
        self.area = area_constant 
        self.a = math.sqrt(self.area)
        self.b = self.area / self.a
        
        # 3. 颜色
        self.color = (255, 165, 0)
        self.border_color = (200, 100, 0)

        # 4. 【核心修复】缓存系统
        # 用来存“正着放的椭圆”，只有变形时才更新
        self.base_surface = None 
        # 用来存“旋转后的椭圆”，只有旋转或变形时才更新
        self.rotated_surface = None
        self.last_drawn_angle = None
        self.needs_redraw = True # 标记：是否需要重绘

    def update_shape(self, change_in_a):
        new_a = self.a + change_in_a
        if 10 < new_a < 100: 
            self.a = new_a
            self.b = self.area / self.a
            self.needs_redraw = True # 形状变了，通知下次重绘

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def set_angle(self, angle_degrees):
        if self.angle != angle_degrees:
            self.angle = angle_degrees
            # 注意：只是旋转并不需要重画 base_surface，但需要重新生成 rotated_surface
            # 这里我们在 draw 里处理

    def draw(self, screen):
        # 1. 检查是否需要更新“基础椭圆图” (只在变形时运行，极省资源)
        if self.needs_redraw or self.base_surface is None:
            length = max(self.a, self.b) * 2
            surf_size = int(length) + 10
            
            # 创建新画布
            self.base_surface = pygame.Surface((surf_size, surf_size), pygame.SRCALPHA)
            
            # 画椭圆
            ellipse_rect = pygame.Rect(0, 0, self.a * 2, self.b * 2)
            ellipse_rect.center = (surf_size // 2, surf_size // 2)
            pygame.draw.ellipse(self.base_surface, self.color, ellipse_rect)
            pygame.draw.ellipse(self.base_surface, self.border_color, ellipse_rect, width=2)
            
            self.needs_redraw = False
            self.last_drawn_angle = None # 基础图变了，旋转图也要强制失效

        # 2. 检查是否需要更新“旋转后的图” (只在角度变化时运行)
        if self.rotated_surface is None or self.angle != self.last_drawn_angle:
            self.rotated_surface = pygame.transform.rotate(self.base_surface, self.angle)
            self.last_drawn_angle = self.angle

        # 3. 直接把缓存好的图贴上去 (这一步非常快，不会闪烁)
        new_rect = self.rotated_surface.get_rect(center=(self.x, self.y))
        screen.blit(self.rotated_surface, new_rect)

    # === 键盘控制逻辑 ===
    def handle_keyboard(self):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]: self.update_shape(1.0)
        if keys[pygame.K_DOWN]: self.update_shape(-1.0)
        if keys[pygame.K_LEFT]: self.angle += 2
        if keys[pygame.K_RIGHT]: self.angle -= 2

    # === 沿路径移动逻辑 ===
    def move_along_path(self, path, path_index, progress, speed, cell_size):
        if path_index < len(path) - 1:
            curr = path[path_index]
            next_node = path[path_index + 1]
            
            c_px = (curr[0] * cell_size + cell_size // 2, curr[1] * cell_size + cell_size // 2)
            n_px = (next_node[0] * cell_size + cell_size // 2, next_node[1] * cell_size + cell_size // 2)
            
            progress += speed
            if progress >= 1.0:
                progress = 0.0
                path_index += 1
                self.set_position(n_px[0], n_px[1])
            else:    
                new_x = c_px[0] + (n_px[0] - c_px[0]) * progress
                new_y = c_px[1] + (n_px[1] - c_px[1]) * progress
                self.set_position(new_x, new_y)
            
        return path_index, progress
    

    def auto_adjust(self, grid, curr_pos, next_pos):
        # 1. 强制锁定角度为 0 (始终保持水平)
        self.set_angle(0)

        # 2. 感知环境 (Sensing)
        # 检查当前格和下一格的墙壁情况
        walls = grid.walls
        
        def check_walls(gx, gy):
            w_left = (gx - 1, gy) in walls
            w_right = (gx + 1, gy) in walls
            w_up = (gx, gy - 1) in walls
            w_down = (gx, gy + 1) in walls
            return w_left, w_right, w_up, w_down

        c_left, c_right, c_up, c_down = check_walls(*curr_pos)
        n_left, n_right, n_up, n_down = check_walls(*next_pos)

        # 3. 变形决策
        # 逻辑：
        # - 如果左右有墙 -> 说明在走垂直通道 -> 必须变瘦 (减小 a, 增大 b)
        # - 如果上下有墙 -> 说明在走水平通道 -> 必须变扁 (增大 a, 减小 b)
        
        squeeze_width = (c_left or c_right) or (n_left or n_right)
        squeeze_height = (c_up or c_down) or (n_up or n_down)
        
        deform_speed = 3.0
        target = math.sqrt(self.area)

        # 优先级判断：
        if squeeze_width and not squeeze_height:
            # 【垂直通道】左右受挤 -> 变瘦 (减小 a)
            # 目标：变得很窄，方便上下穿行
            if self.a > 20: 
                self.update_shape(-deform_speed)
                
        elif squeeze_height and not squeeze_width:
            # 【水平通道】上下受挤 -> 变扁 (增大 a)
            # 目标：变得很矮，方便左右穿行
            if self.a < 75:
                self.update_shape(deform_speed)
                
        elif squeeze_width and squeeze_height:
            # 【死角/十字路口】四面楚歌 -> 恢复圆球保平安
            if abs(self.a - target) > 1:
                d = -1 if self.a > target else 1
                self.update_shape(d * deform_speed)
        
        else:
            # 【空旷区域】恢复正圆
            if abs(self.a - target) > 1:
                d = -1 if self.a > target else 1
                self.update_shape(d * deform_speed)