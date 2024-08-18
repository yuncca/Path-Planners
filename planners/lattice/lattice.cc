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
 * File: lattice.cc
 */
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

// 定义点结构体
struct Point {
  double x, y;
  Point(double _x, double _y) : x(_x), y(_y) {}
};

// 计算欧几里得距离
double euclideanDistance(const Point &p1, const Point &p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// 生成样条路径
vector<vector<Point>> generateLatticePaths(Point start, Point goal,
                                           int num_paths, double path_length) {
  vector<vector<Point>> paths;

  double delta_y = (goal.y - start.y) / (num_paths - 1);
  double delta_x = path_length / 5;  // 样条的分段长度

  for (int i = 0; i < num_paths; i++) {
    vector<Point> path;
    double offset_y = start.y + i * delta_y;

    for (int j = 0; j <= 5; j++) {
      double x = start.x + j * delta_x;
      double y =
          offset_y + (goal.y - offset_y) * (x - start.x) / (goal.x - start.x);
      path.push_back(Point(x, y));
    }

    paths.push_back(path);
  }

  return paths;
}

// 计算路径代价（例如与目标点的距离和路径的曲率）
double calculatePathCost(const vector<Point> &path, const Point &goal) {
  double cost = 0.0;

  // 路径终点与目标点的距离
  cost += euclideanDistance(path.back(), goal);

  // 路径曲率代价
  for (size_t i = 1; i < path.size(); i++) {
    cost += euclideanDistance(path[i], path[i - 1]);
  }

  return cost;
}

// 选择代价最低的路径
vector<Point> selectOptimalPath(const vector<vector<Point>> &paths,
                                const Point &goal) {
  double min_cost = numeric_limits<double>::max();
  vector<Point> optimal_path;

  for (const auto &path : paths) {
    double cost = calculatePathCost(path, goal);
    if (cost < min_cost) {
      min_cost = cost;
      optimal_path = path;
    }
  }

  return optimal_path;
}

int main() {
  Point start(0.0, 0.0);
  Point goal(10.0, 5.0);

  int num_paths = 5;
  double path_length = 10.0;

  vector<vector<Point>> lattice_paths =
      generateLatticePaths(start, goal, num_paths, path_length);
  vector<Point> optimal_path = selectOptimalPath(lattice_paths, goal);

  // 输出最优路径
  if (!optimal_path.empty()) {
    cout << "Optimal Path:" << endl;
    for (const auto &p : optimal_path) {
      cout << "(" << p.x << ", " << p.y << ")" << endl;
    }
  } else {
    cout << "No optimal path found!" << endl;
  }

  return 0;
}
