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
 * File: Vec2d.h
 */
#pragma once
#include <cstdint>

namespace pathplan {
namespace common {
class Vec2d {
 public:
  Vec2d(const int32_t x, const int32_t y) : x_(x), y_(y) {}

  virtual ~Vec2d() = default;

  int32_t x() const { return x_; }

  int32_t y() const { return y_; }

  void set_x(int32_t x) { x_ = x; }

  void set_y(int32_t y) { y_ = y; }

  bool operator==(const Vec2d &other) const;

 private:
  int32_t x_ = 0;
  int32_t y_ = 0;
};

}  // namespace common
}  // namespace pathplan
