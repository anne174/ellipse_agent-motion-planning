from __future__ import annotations
#引入类型提示相关的模块
from typing import Protocol, Iterator, Tuple, TypeVar, Optional
#Protocol（协议）；Iterator(迭代器)；Tuple(元组)表示一个固定长度、不可变的序列；
#TypeVar(类型变量)；Optional(可选)表示这个变量既可以是某种类型，也可以是 None；
T = TypeVar('T')
#定义一个通用的类型变量，名字叫 T
Location = TypeVar('Location')
#定义一个特定的类型变量，名字叫 Location

#定义Graph类，继承自Protocol,(鸭子类型)
class Graph(Protocol):
    #neighbors函数：输入：id(Location类型)
    #输出：list[Location],由位置组成的列表
    def neighbors(self, id: Location) -> list[Location]: pass

#SimpleGraph类
class SimpleGraph:
    #构造函数，创建一个新的图时，这个函数会自动运行
    def __init__(self):
        #初始化一个空字典edges,dict[(Location一个节点)Key,（list邻居节点)Value]
        self.edges: dict[Location, list[Location]] = {}
    #neighbors方法，一个合法的Graph
    def neighbors(self, id: Location) -> list[Location]:
        #依据id去self.edges字典里查找，返回对应的列表
        return self.edges[id]
#样例
example_graph = SimpleGraph()
example_graph.edges = {
    'A': ['B'],
    'B': ['C'],
    'C': ['B', 'D', 'F'],
    'D': ['C', 'E'],
    'E': ['F'],
    'F': [],
}

#collections:Container datatypes"（容器数据类型）,提供了一些特殊的、高性能的“容器”
import collections

#Queue类：FIFO
class Queue:
    def __init__(self):
        #双端队列
        self.elements = collections.deque()
    #empty方法：检查队列里是否还有东西.-> bool提示预期返回一个布尔值
    def empty(self) -> bool:
        #如果 self.elements 是空的，not 运算会返回 True
        return not self.elements
    #put方法：入队（append:从尾部加）
    def put(self, x: T):
        self.elements.append(x)
    #get方法：出队（popleft:移除并返回最前面的元素）
    def get(self) -> T:
        return self.elements.popleft()

# 处理方格网格，在“一维数组索引”和“二维矩阵坐标”之间进行转换
#id = 13：
#x 坐标：13 % 10 = 3（余数是3，所以在第3列）
#y 坐标：13 // 10 = 1（整除是1，所以在第1行）
def from_id_width(id, width):
    return (id % width, id // width)

#控制台print地图
#graph: 地图对象（用来查墙壁）。
#id: 当前正在画的格子的坐标 (x, y)。
#style: 一个字典，包含了绘图的配置信息
def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " @ "
    if 'start' in style and id == style['start']: r = " A "
    if 'goal' in style and id == style['goal']:   r = " Z "
    if id in graph.walls: r = "###"
    return r

#print
#**style：把所有传进来的额外关键字参数，打包成一个名叫 style 的字典，调用时生成
def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)

# 一个包含坐标元组的列表，随后会被赋值给 g.walls
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
#定义类型别名
GridLocation = Tuple[int, int]

#二维方格网格地图
class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: list[GridLocation] = []
    #边界检查
    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    #通行检查
    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls
    #获取邻居
    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1),(x+1,y+1),(x-1,y+1),(x-1,y-1),(x+1,y-1)]#增加为8方向的移动
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

#定义标准接口
class WeightedGraph(Graph):
    def cost(self, from_id: Location, to_id: Location) -> float: pass
#权重方格图，继承SquareGrid
class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: dict[GridLocation, float] = {}
    #计算代价
    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)

def inflate_walls(graph,radius):
    """让墙壁向外便后一圈：遍历现有墙壁，把它们周围的邻居也变成墙壁"""
    original_walls=set(graph.walls)
    new_walls=set(graph.walls)
    for(wx,wy) in original_walls :
        """遍历墙壁周围radius范围内的格子"""
        for dx in range(-radius,radius+1):
            for dy in range(-radius,radius+1):
                if dx ==0 and dy ==0:
                    continue
                neighbor=(wx+dx,wy+dy)
                if graph.in_bounds(neighbor):
                    new_walls.add(neighbor)
    return list(new_walls)

#10*10地图
diagram4 = GridWithWeights(10, 10)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}

#20*20地图
diagram_20 = GridWithWeights(20, 20)
diagram_20.walls = [
    (10, 3), (10, 4), (10, 5), (10, 6), (10, 7), (10, 8), 
    (10, 11), (10, 12), (10, 13), (10, 14), (10, 15), (10, 16), 
    (2, 3), (2, 7), (3, 3), (3, 7), (4, 3), (4, 7), (5, 3), 
    (5, 7), (6, 3), (6, 7), (2, 3), (2, 4), (2, 5), (2, 6), 
    (2, 7), (14, 12), (14, 18), (15, 12), (15, 18), (16, 12), 
    (16, 18), (17, 12), (17, 18), (18, 12), (18, 18), (14, 12), 
    (18, 12), (14, 13), (18, 13), (14, 14), (18, 14), (14, 15), 
    (14, 16), (18, 16), (14, 17), (18, 17), (14, 18), (18, 18), 
    (5, 12), (6, 13), (4, 14), (15, 5), (16, 4), (17, 6), 
    (8, 16), (9, 17)
]
inflate_walls(diagram_20,1)
# if (3, 2) in diagram_20.walls: diagram_20.walls.remove((3, 2))
# if (2, 2) in diagram_20.walls: diagram_20.walls.remove((2, 2))
# if (4, 2) in diagram_20.walls: diagram_20.walls.remove((4, 2))
# if (4, 1) in diagram_20.walls: diagram_20.walls.remove((4, 1))
# if (2, 1) in diagram_20.walls: diagram_20.walls.remove((2, 1))
# if (3, 0) in diagram_20.walls: diagram_20.walls.remove((3, 0))
# if (2, 0) in diagram_20.walls: diagram_20.walls.remove((2, 0))
# if (4, 0) in diagram_20.walls: diagram_20.walls.remove((4, 0))

diagram_20.weights = {loc: 5 for loc in [
    (12, 1), (12, 2), (12, 3), (12, 4), (12, 5), 
    (13, 1), (13, 2), (13, 3), (13, 4), (13, 5), 
    (14, 1), (14, 2), (14, 3), (14, 4), (14, 5), 
    (15, 1), (15, 2), (15, 3), (15, 4), (15, 5), 
    (16, 1), (16, 2), (16, 3), (16, 4), (16, 5), 
    (17, 1), (17, 2), (17, 3), (17, 4), (17, 5)
]}

import heapq

#优先队列
class PriorityQueue:
    def __init__(self):
        self.elements: list[tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

#Dijkstra算法
def dijkstra_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

#A*算法
def a_star_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

#广度优先搜索（BFS）
def breadth_first_search(graph: Graph, start: Location, goal: Location):
    frontier = Queue()
    frontier.put(start)
    came_from: dict[Location, Optional[Location]] = {}
    came_from[start] = None
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
    
    return came_from

#从终点回溯到起点，从而重构出一条完整的路径
def reconstruct_path(came_from: dict[Location, Location],
                     start: Location, goal: Location) -> list[Location]:

    current: Location = goal
    path: list[Location] = []
    if goal not in came_from: # no path was found
        return []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path
#测试死路场景
diagram_nopath = GridWithWeights(10, 10)
diagram_nopath.walls = [(5, row) for row in range(10)]

#曼哈顿距离
def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

#允许自定义搜索邻居的顺序
class SquareGridNeighborOrder(SquareGrid):
    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x + dx, y + dy) for (dx, dy) in self.NEIGHBOR_ORDER]
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return list(results)

#验证和展示“搜索邻居的顺序”是如何影响路径规划结果的
def test_with_custom_order(neighbor_order):
    if neighbor_order:
        g = SquareGridNeighborOrder(30, 15)
        g.NEIGHBOR_ORDER = neighbor_order
    else:
        g = SquareGrid(30, 15)
    g.walls = DIAGRAM1_WALLS
    start, goal = (8, 7), (27, 2)
    came_from = breadth_first_search(g, start, goal)
    draw_grid(g, path=reconstruct_path(came_from, start=start, goal=goal),
              point_to=came_from, start=start, goal=goal)

#打破平局，让路径更直（未使用）
class GridWithAdjustedWeights(GridWithWeights):
    def cost(self, from_node, to_node):
        prev_cost = super().cost(from_node, to_node)
        (x1, y1) = from_node
        (x2, y2) = to_node
        if  x1!=x2 and y1!=y2:
            return prev_cost*1.141
        else:
            return prev_cost


