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
 * File: astar.cc
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

#include "common/vec2d.h"

namespace pathplan {
namespace planner {

using namespace ::common::Vec2d;
using namespace std;
struct Node {
  Vec2d point;
  int g_cost, h_cost, f_cost;

  Node(Vec2d _point, int _g_cost, int _h_cost)
      : point(_point), g_cost(_g_cost), h_cost(_h_cost) {
    f_cost = g_cost + h_cost;
  }

  // 优先队列需要根据f_cost排序
  bool operator<(const Node &other) const {
    return f_cost > other.f_cost;  // 逆序，f_cost小的优先
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

// 计算启发式函数h值（曼哈顿距离）
int calculateHeuristic(Vec2d p1, Vec2d p2) {
  return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

// A*路径规划算法
vector<Vec2d> aStarPathPlanning(const vector<vector<int>> &grid, Vec2d start,
                                Vec2d goal) {
  int rows = grid.size();
  int cols = grid[0].size();

  vector<vector<int>> g_cost(rows,
                             vector<int>(cols, numeric_limits<int>::max()));
  vector<vector<Vec2d>> parent(rows, vector<Vec2d>(cols, Vec2d(-1, -1)));
  priority_queue<Node> pq;

  g_cost[start.x][start.y] = 0;
  int h_cost = calculateHeuristic(start, goal);
  pq.push(Node(start, 0, h_cost));

  while (!pq.empty()) {
    Node current = pq.top();
    pq.pop();

    Vec2d p = current.point;

    // 如果当前点是目标点，构造路径并返回
    if (p == goal) {
      vector<Vec2d> path;
      for (Vec2d at = goal; at.x != -1 && at.y != -1; at = parent[at.x][at.y]) {
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
        int new_g_cost = g_cost[p.x][p.y] + 1;  // 假设每个格子的移动代价为1
        int new_h_cost = calculateHeuristic(Vec2d(newX, newY), goal);
        int new_f_cost = new_g_cost + new_h_cost;

        if (new_g_cost < g_cost[newX][newY]) {
          g_cost[newX][newY] = new_g_cost;
          parent[newX][newY] = p;
          pq.push(Node(Vec2d(newX, newY), new_g_cost, new_h_cost));
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

  Vec2d start(0, 0);
  Vec2d goal(4, 4);

  vector<Vec2d> path = aStarPathPlanning(grid, start, goal);

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

}  // namespace planner

}  // namespace pathplan
