/**
 * MIT License
 *
 * Copyright (c) 2024 YunCC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * File: bfs_planner.cc
 */

#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

// 定义点结构体
struct Point {
  int x, y;
  Point(int _x, int _y) : x(_x), y(_y) {}
};

// 定义一个方向数组，用来表示四个可能的移动方向（上、下、左、右）
int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};

// 检查坐标是否合法（是否在地图范围内）
bool isValid(int x, int y, int rows, int cols,
             const vector<vector<int>> &grid) {
  return x >= 0 && x < rows && y >= 0 && y < cols && grid[x][y] == 0;
}

// BFS路径规划算法
vector<Point> bfsPathPlanning(const vector<vector<int>> &grid, Point start,
                              Point goal) {
  int rows = grid.size();
  int cols = grid[0].size();

  vector<vector<bool>> visited(rows, vector<bool>(cols, false));
  vector<vector<Point>> parent(rows, vector<Point>(cols, Point(-1, -1)));

  queue<Point> q;
  q.push(start);
  visited[start.x][start.y] = true;

  while (!q.empty()) {
    Point current = q.front();
    q.pop();

    // 如果找到了目标点，构造路径并返回
    if (current.x == goal.x && current.y == goal.y) {
      vector<Point> path;
      for (Point at = goal; at.x != -1 && at.y != -1; at = parent[at.x][at.y]) {
        path.push_back(at);
      }
      reverse(path.begin(), path.end());
      return path;
    }

    // 扩展当前节点的四个邻居节点
    for (int i = 0; i < 4; i++) {
      int newX = current.x + dx[i];
      int newY = current.y + dy[i];

      if (isValid(newX, newY, rows, cols, grid) && !visited[newX][newY]) {
        q.push(Point(newX, newY));
        visited[newX][newY] = true;
        parent[newX][newY] = current;
      }
    }
  }

  // 如果没有找到路径，返回空路径
  return {};
}

int main() {
  // 0表示可以通过的点，1表示障碍物
  vector<vector<int>> grid = {{0, 1, 0, 0, 0},
                              {0, 1, 0, 1, 0},
                              {0, 0, 0, 1, 0},
                              {0, 1, 0, 0, 0},
                              {0, 0, 0, 1, 0}};

  Point start(0, 0);
  Point goal(4, 4);

  vector<Point> path = bfsPathPlanning(grid, start, goal);

  if (!path.empty()) {
    cout << "Path found:" << endl;
    for (const auto &p : path) {
      cout << "(" << p.x << ", " << p.y << ") ";
    }
    cout << endl;
  } else {
    cout << "No path found!" << endl;
  }

  return 0;
}
