#include "planbench/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planbench {
namespace {

struct Interval {
  double lo = 0.0;
  double hi = 0.0;
};

Interval project(const Vec2* points, int count, double ax, double ay) {
  Interval span{points[0].x * ax + points[0].y * ay, 0.0};
  span.hi = span.lo;
  for (int i = 1; i < count; ++i) {
    const double value = points[i].x * ax + points[i].y * ay;
    span.lo = std::min(span.lo, value);
    span.hi = std::max(span.hi, value);
  }
  return span;
}

bool separated(Interval a, Interval b) {
  return a.hi < b.lo || b.hi < a.lo;
}

}  // namespace

OccupancyGrid::OccupancyGrid(int width, int height, double resolution) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("occupancy grid dimensions must be positive");
  }
  if (!(resolution > 0.0) || !std::isfinite(resolution)) {
    throw std::invalid_argument("occupancy grid resolution must be positive");
  }
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  cells_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
}

bool OccupancyGrid::inBounds(int ix, int iy) const {
  return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
}

bool OccupancyGrid::occupied(int ix, int iy) const {
  if (!inBounds(ix, iy)) {
    return true;
  }
  return cells_[static_cast<std::size_t>(index(ix, iy))] != 0;
}

void OccupancyGrid::setOccupied(int ix, int iy, bool value) {
  if (!inBounds(ix, iy)) {
    throw std::out_of_range("cell is outside the occupancy grid");
  }
  cells_[static_cast<std::size_t>(index(ix, iy))] = value ? 1 : 0;
}

bool OccupancyGrid::worldToCell(double x, double y, int& ix, int& iy) const {
  if (!(x >= 0.0 && y >= 0.0 && x < widthMeters() && y < heightMeters())) {
    return false;
  }
  ix = static_cast<int>(std::floor(x / resolution_));
  iy = static_cast<int>(std::floor(y / resolution_));
  if (ix >= width_) {
    ix = width_ - 1;
  }
  if (iy >= height_) {
    iy = height_ - 1;
  }
  return true;
}

Vec2 OccupancyGrid::cellCenter(int ix, int iy) const {
  if (!inBounds(ix, iy)) {
    throw std::out_of_range("cell is outside the occupancy grid");
  }
  return Vec2{(static_cast<double>(ix) + 0.5) * resolution_, (static_cast<double>(iy) + 0.5) * resolution_};
}

bool OccupancyGrid::pointFree(double x, double y) const {
  int ix = 0;
  int iy = 0;
  if (!worldToCell(x, y, ix, iy)) {
    return false;
  }
  return !occupied(ix, iy);
}

bool OccupancyGrid::segmentFree(Vec2 a, Vec2 b) const {
  if (!pointFree(a.x, a.y) || !pointFree(b.x, b.y)) {
    return false;
  }

  int x = 0;
  int y = 0;
  int x_end = 0;
  int y_end = 0;
  if (!worldToCell(a.x, a.y, x, y) || !worldToCell(b.x, b.y, x_end, y_end)) {
    return false;
  }

  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const int step_x = (dx > 0.0) - (dx < 0.0);
  const int step_y = (dy > 0.0) - (dy < 0.0);
  const double inf = std::numeric_limits<double>::infinity();
  const double t_delta_x = step_x == 0 ? inf : std::abs(resolution_ / dx);
  const double t_delta_y = step_y == 0 ? inf : std::abs(resolution_ / dy);

  double t_max_x = inf;
  double t_max_y = inf;
  if (step_x > 0) {
    t_max_x = ((static_cast<double>(x) + 1.0) * resolution_ - a.x) / dx;
  } else if (step_x < 0) {
    t_max_x = (static_cast<double>(x) * resolution_ - a.x) / dx;
  }
  if (step_y > 0) {
    t_max_y = ((static_cast<double>(y) + 1.0) * resolution_ - a.y) / dy;
  } else if (step_y < 0) {
    t_max_y = (static_cast<double>(y) * resolution_ - a.y) / dy;
  }

  const int limit = width_ + height_ + 4;
  for (int step = 0; step < limit; ++step) {
    if (!inBounds(x, y) || occupied(x, y)) {
      return false;
    }
    if (x == x_end && y == y_end) {
      return true;
    }
    if (t_max_x < t_max_y) {
      x += step_x;
      t_max_x += t_delta_x;
    } else if (t_max_y < t_max_x) {
      y += step_y;
      t_max_y += t_delta_y;
    } else {
      const int side_x = x + step_x;
      const int side_y = y + step_y;
      if (inBounds(side_x, y) && occupied(side_x, y)) {
        return false;
      }
      if (inBounds(x, side_y) && occupied(x, side_y)) {
        return false;
      }
      x = side_x;
      y = side_y;
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    }
  }
  return false;
}

bool OccupancyGrid::rectangleFree(double x, double y, double theta, double length, double width) const {
  if (!(length > 0.0) || !(width > 0.0)) {
    return false;
  }
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  const double half_length = 0.5 * length;
  const double half_width = 0.5 * width;
  const double local_x[4] = {half_length, half_length, -half_length, -half_length};
  const double local_y[4] = {half_width, -half_width, half_width, -half_width};

  Vec2 corners[4];
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < 4; ++i) {
    corners[i].x = x + c * local_x[i] - s * local_y[i];
    corners[i].y = y + s * local_x[i] + c * local_y[i];
    min_x = std::min(min_x, corners[i].x);
    min_y = std::min(min_y, corners[i].y);
    max_x = std::max(max_x, corners[i].x);
    max_y = std::max(max_y, corners[i].y);
  }
  if (min_x < 0.0 || min_y < 0.0 || max_x >= widthMeters() || max_y >= heightMeters()) {
    return false;
  }

  const int ix0 = std::max(0, static_cast<int>(std::floor(min_x / resolution_)));
  const int iy0 = std::max(0, static_cast<int>(std::floor(min_y / resolution_)));
  const int ix1 = std::min(width_ - 1, static_cast<int>(std::floor((max_x - 1e-12) / resolution_)));
  const int iy1 = std::min(height_ - 1, static_cast<int>(std::floor((max_y - 1e-12) / resolution_)));

  const Vec2 axes[2] = {Vec2{c, s}, Vec2{-s, c}};
  for (int iy = iy0; iy <= iy1; ++iy) {
    for (int ix = ix0; ix <= ix1; ++ix) {
      if (!occupied(ix, iy)) {
        continue;
      }
      const double x0 = static_cast<double>(ix) * resolution_;
      const double y0 = static_cast<double>(iy) * resolution_;
      const Vec2 cell[4] = {
          Vec2{x0, y0},
          Vec2{x0 + resolution_, y0},
          Vec2{x0 + resolution_, y0 + resolution_},
          Vec2{x0, y0 + resolution_},
      };
      if (separated(project(cell, 4, 1.0, 0.0), project(corners, 4, 1.0, 0.0)) ||
          separated(project(cell, 4, 0.0, 1.0), project(corners, 4, 0.0, 1.0))) {
        continue;
      }
      bool apart = false;
      for (const Vec2& axis : axes) {
        if (separated(project(cell, 4, axis.x, axis.y), project(corners, 4, axis.x, axis.y))) {
          apart = true;
          break;
        }
      }
      if (!apart) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace planbench
