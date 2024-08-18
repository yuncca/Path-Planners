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
 * File: dijkstra.cc
 */

#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

using namespace std;

// 定义点结构体
struct Point {
  int x, y;
  Point(int _x, int _y) : x(_x), y(_y) {}

  // 重载 == 运算符，便于比较
  bool operator==(const Point &other) const {
    return x == other.x && y == other.y;
  }
};

// 定义节点结构体，用于优先队列
struct Node {
  Point point;
  int cost;

  Node(Point _point, int _cost) : point(_point), cost(_cost) {}

  // 优先队列需要根据cost排序
  bool operator<(const Node &other) const {
    return cost > other.cost;  // 逆序，cost小的优先
  }
};

// 定义一个方向数组，用来表示四个可能的移动方向（上、下、左、右）
int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};

// 检查坐标是否合法（是否在地图范围内）
bool isValid(int x, int y, int rows, int cols,
             const vector<vector<int>> &grid) {
  return x >= 0 && x < rows && y >= 0 && y < cols && grid[x][y] == 0;
}

// Dijkstra路径规划算法
vector<Point> dijkstraPathPlanning(const vector<vector<int>> &grid, Point start,
                                   Point goal) {
  int rows = grid.size();
  int cols = grid[0].size();

  vector<vector<int>> dist(rows, vector<int>(cols, numeric_limits<int>::max()));
  vector<vector<Point>> parent(rows, vector<Point>(cols, Point(-1, -1)));
  priority_queue<Node> pq;

  dist[start.x][start.y] = 0;
  pq.push(Node(start, 0));

  while (!pq.empty()) {
    Node current = pq.top();
    pq.pop();

    Point p = current.point;

    // 如果当前点是目标点，构造路径并返回
    if (p == goal) {
      vector<Point> path;
      for (Point at = goal; at.x != -1 && at.y != -1; at = parent[at.x][at.y]) {
        path.push_back(at);
      }
      reverse(path.begin(), path.end());
      return path;
    }

    // 扩展当前节点的四个邻居节点
    for (int i = 0; i < 4; i++) {
      int newX = p.x + dx[i];
      int newY = p.y + dy[i];

      if (isValid(newX, newY, rows, cols, grid)) {
        int newCost = dist[p.x][p.y] + 1;  // 假设每个格子的移动代价为1
        if (newCost < dist[newX][newY]) {
          dist[newX][newY] = newCost;
          parent[newX][newY] = p;
          pq.push(Node(Point(newX, newY), newCost));
        }
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

  vector<Point> path = dijkstraPathPlanning(grid, start, goal);

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
