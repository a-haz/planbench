#pragma once

#include <cmath>
#include <vector>

namespace planbench {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Pose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

inline double distance(Vec2 a, Vec2 b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double polylineLength(const std::vector<Vec2>& path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += distance(path[i - 1], path[i]);
  }
  return length;
}

struct PlanResult {
  bool success = false;
  std::vector<Vec2> path;
  double length = 0.0;
  int expansions = 0;
};

}  // namespace planbench
