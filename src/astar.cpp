#include "planbench/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace planbench {

PlanResult AStarPlanner::plan(const OccupancyGrid& grid, Vec2 start, Vec2 goal) const {
  PlanResult result;
  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  if (!grid.worldToCell(start.x, start.y, sx, sy) || !grid.worldToCell(goal.x, goal.y, gx, gy)) {
    return result;
  }
  if (grid.occupied(sx, sy) || grid.occupied(gx, gy)) {
    return result;
  }

  const int width = grid.width();
  const int height = grid.height();
  const int cell_count = width * height;
  const double resolution = grid.resolution();
  const double diagonal = resolution * std::sqrt(2.0);
  const double inf = std::numeric_limits<double>::infinity();

  std::vector<double> g_score(static_cast<std::size_t>(cell_count), inf);
  std::vector<int> parent(static_cast<std::size_t>(cell_count), -1);
  std::vector<char> closed(static_cast<std::size_t>(cell_count), 0);

  const auto id_of = [width](int x, int y) { return y * width + x; };
  const auto heuristic = [&](int x, int y) {
    const int dx = std::abs(x - gx);
    const int dy = std::abs(y - gy);
    const int diagonal_steps = std::min(dx, dy);
    const int cardinal_steps = dx + dy - 2 * diagonal_steps;
    return static_cast<double>(cardinal_steps) * resolution + static_cast<double>(diagonal_steps) * diagonal;
  };

  struct OpenNode {
    double f = 0.0;
    double g = 0.0;
    int sequence = 0;
    int x = 0;
    int y = 0;
    bool operator<(const OpenNode& other) const {
      if (f != other.f) {
        return f > other.f;
      }
      if (g != other.g) {
        return g < other.g;
      }
      return sequence > other.sequence;
    }
  };

  std::priority_queue<OpenNode> open;
  const int start_id = id_of(sx, sy);
  g_score[static_cast<std::size_t>(start_id)] = 0.0;
  open.push(OpenNode{heuristic(sx, sy), 0.0, 0, sx, sy});
  int sequence = 1;

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const OpenNode current = open.top();
    open.pop();
    const int current_id = id_of(current.x, current.y);
    if (closed[static_cast<std::size_t>(current_id)]) {
      continue;
    }
    if (current.g != g_score[static_cast<std::size_t>(current_id)]) {
      continue;
    }
    closed[static_cast<std::size_t>(current_id)] = 1;
    ++result.expansions;

    if (current.x == gx && current.y == gy) {
      std::vector<Vec2> reversed;
      int x = gx;
      int y = gy;
      while (true) {
        reversed.push_back(grid.cellCenter(x, y));
        const int parent_id = parent[static_cast<std::size_t>(id_of(x, y))];
        if (parent_id < 0) {
          break;
        }
        x = parent_id % width;
        y = parent_id / width;
      }
      std::reverse(reversed.begin(), reversed.end());
      result.success = true;
      result.length = g_score[static_cast<std::size_t>(current_id)];
      result.path = std::move(reversed);
      return result;
    }

    for (int k = 0; k < 8; ++k) {
      const int nx = current.x + dxs[k];
      const int ny = current.y + dys[k];
      if (!grid.inBounds(nx, ny) || grid.occupied(nx, ny)) {
        continue;
      }
      const int neighbor_id = id_of(nx, ny);
      if (closed[static_cast<std::size_t>(neighbor_id)]) {
        continue;
      }
      const bool diagonal_step = dxs[k] != 0 && dys[k] != 0;
      if (diagonal_step) {
        if (grid.occupied(current.x + dxs[k], current.y) || grid.occupied(current.x, current.y + dys[k])) {
          continue;
        }
      }
      const double step_cost = diagonal_step ? diagonal : resolution;
      const double tentative = current.g + step_cost;
      if (tentative + 1e-12 < g_score[static_cast<std::size_t>(neighbor_id)]) {
        g_score[static_cast<std::size_t>(neighbor_id)] = tentative;
        parent[static_cast<std::size_t>(neighbor_id)] = current_id;
        open.push(OpenNode{tentative + heuristic(nx, ny), tentative, sequence++, nx, ny});
      }
    }
  }
  return result;
}

}  // namespace planbench
