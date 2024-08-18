import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors

# BFS搜索过程的可视化绘图函数（带箭头连接）
def plot_bfs_with_arrows(grid, path, visited_points, parent_map):
    rows, cols = len(grid), len(grid[0])

    # 创建一个颜色映射
    cmap = colors.ListedColormap(['white', 'black', 'red', 'yellow'])
    bounds = [0, 1, 2, 3, 4]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    # 创建一个矩阵用于展示搜索过程
    search_grid = np.zeros((rows, cols))

    # 标记路径、已访问的点
    for point in visited_points:
        search_grid[point.x][point.y] = 2  # 已访问点标记为2（红色）

    for point in path:
        search_grid[point.x][point.y] = 3  # 路径点标记为3（黄色）

    search_grid[grid == 1] = 1  # 障碍物标记为1（黑色）

    # 绘制图像
    plt.figure(figsize=(6, 6))
    plt.imshow(search_grid, cmap=cmap, norm=norm)

    # 标注起点和终点
    plt.text(0, 0, 'Start', va='center', ha='center', color='blue', fontsize=12)
    plt.text(cols-1, rows-1, 'Goal', va='center', ha='center', color='blue', fontsize=12)

    # 添加箭头，连接父节点和子节点
    for point in visited_points:
        if parent_map[point.x][point.y].x != -1:  # 有父节点的点
            parent = parent_map[point.x][point.y]
            dx = point.x - parent.x
            dy = point.y - parent.y
            plt.quiver(parent.y, parent.x, dy, dx, angles='xy', scale_units='xy', scale=1, color='blue', width=0.005)

    # 添加网格线
    plt.grid(which='both', color='black', linestyle='-', linewidth=1)
    plt.xticks(np.arange(-.5, cols, 1), [])
    plt.yticks(np.arange(-.5, rows, 1), [])
    plt.gca().set_xticks(np.arange(-.5, cols, 1), minor=True)
    plt.gca().set_yticks(np.arange(-.5, rows, 1), minor=True)
    plt.gca().grid(which="minor", color="black", linestyle='-', linewidth=1)

    plt.show()

# 示例使用
if __name__ == "__main__":
    grid = np.array([
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0]
    ])

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    path = [Point(0, 0), Point(1, 0), Point(2, 0), Point(2, 1), Point(2, 2), Point(3, 2), Point(4, 2), Point(4, 3), Point(4, 4)]
    visited_points = [Point(0, 0), Point(1, 0), Point(2, 0), Point(2, 1), Point(1, 2), Point(0, 2), Point(0, 3), Point(0, 4), Point(2, 2), Point(3, 2), Point(4, 2), Point(3, 3), Point(3, 4), Point(4, 4)]
    parent_map = [[Point(-1, -1) for _ in range(5)] for _ in range(5)]
    
    # 构造parent_map，模拟父节点关系
    parent_map[1][0] = Point(0, 0)
    parent_map[2][0] = Point(1, 0)
    parent_map[2][1] = Point(2, 0)
    parent_map[2][2] = Point(2, 1)
    parent_map[3][2] = Point(2, 2)
    parent_map[4][2] = Point(3, 2)
    parent_map[4][3] = Point(4, 2)
    parent_map[4][4] = Point(4, 3)
    
    plot_bfs_with_arrows(grid, path, visited_points, parent_map)
