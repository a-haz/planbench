#include "planbench/state_lattice.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace planbench {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kHeadings = 4;

enum class PrimitiveKind { Straight, Left, Right };

struct Primitive {
  int dx = 0;
  int dy = 0;
  int heading_delta = 0;
  double length = 0.0;
  PrimitiveKind kind = PrimitiveKind::Straight;
  int cells = 0;
};

struct Sample {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

void rotateCell(int dx, int dy, int heading, int& out_x, int& out_y) {
  switch (heading & 3) {
    case 0:
      out_x = dx;
      out_y = dy;
      break;
    case 1:
      out_x = -dy;
      out_y = dx;
      break;
    case 2:
      out_x = -dx;
      out_y = -dy;
      break;
    default:
      out_x = dy;
      out_y = -dx;
      break;
  }
}

Sample interpolate(Vec2 origin, double heading, const Primitive& primitive, double radius, double u) {
  const double theta0 = heading;
  if (primitive.kind == PrimitiveKind::Straight) {
    return Sample{origin.x + u * primitive.length * std::cos(theta0),
                  origin.y + u * primitive.length * std::sin(theta0), theta0};
  }
  const double phi = u * (kPi * 0.5);
  const double sign = primitive.kind == PrimitiveKind::Left ? 1.0 : -1.0;
  const double local_x = radius * std::sin(phi);
  const double local_y = sign * radius * (1.0 - std::cos(phi));
  const double c = std::cos(theta0);
  const double s = std::sin(theta0);
  return Sample{origin.x + c * local_x - s * local_y, origin.y + s * local_x + c * local_y, theta0 + sign * phi};
}

bool primitiveFree(const OccupancyGrid& grid, Vec2 origin, double heading, const Primitive& primitive,
                   double radius, double body_length, double body_width) {
  const double spacing = grid.resolution() * 0.25;
  const int steps = std::max(1, static_cast<int>(std::ceil(primitive.length / spacing)));
  for (int i = 0; i <= steps; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(steps);
    const Sample sample = interpolate(origin, heading, primitive, radius, u);
    if (!grid.rectangleFree(sample.x, sample.y, sample.theta, body_length, body_width)) {
      return false;
    }
  }
  return true;
}

std::vector<Primitive> buildPrimitives(int radius_cells, double resolution) {
  std::vector<Primitive> primitives;
  std::vector<int> straights = {1, 2, radius_cells};
  std::sort(straights.begin(), straights.end());
  straights.erase(std::unique(straights.begin(), straights.end()), straights.end());
  for (int cells : straights) {
    if (cells <= 0) {
      continue;
    }
    Primitive straight;
    straight.dx = cells;
    straight.dy = 0;
    straight.heading_delta = 0;
    straight.length = static_cast<double>(cells) * resolution;
    straight.kind = PrimitiveKind::Straight;
    straight.cells = cells;
    primitives.push_back(straight);
  }
  const double radius = static_cast<double>(radius_cells) * resolution;
  Primitive left;
  left.dx = radius_cells;
  left.dy = radius_cells;
  left.heading_delta = 1;
  left.length = radius * kPi * 0.5;
  left.kind = PrimitiveKind::Left;
  left.cells = radius_cells;
  primitives.push_back(left);

  Primitive right = left;
  right.dy = -radius_cells;
  right.heading_delta = -1;
  right.kind = PrimitiveKind::Right;
  primitives.push_back(right);
  return primitives;
}

}  // namespace

LatticeResult StateLatticePlanner::plan(const OccupancyGrid& grid, Vec2 start, Vec2 goal,
                                         const LatticeParams& params) const {
  LatticeResult result;
  result.body_length_m = params.body_length_m;
  result.body_width_m = params.body_width_m;
  if (!(params.body_length_m > 0.0) || !(params.body_width_m > 0.0) || !(params.turning_radius_m > 0.0)) {
    return result;
  }

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

  const int radius_cells = std::max(1, static_cast<int>(std::lround(params.turning_radius_m / grid.resolution())));
  const double radius = static_cast<double>(radius_cells) * grid.resolution();
  result.turning_radius_m = radius;
  const std::vector<Primitive> primitives = buildPrimitives(radius_cells, grid.resolution());

  const Vec2 start_center = grid.cellCenter(sx, sy);
  if (!grid.rectangleFree(start_center.x, start_center.y, 0.0, params.body_length_m, params.body_width_m)) {
    return result;
  }

  const int width = grid.width();
  const int height = grid.height();
  const int state_count = width * height * kHeadings;
  const auto state_id = [width](int x, int y, int heading) {
    return (y * width + x) * kHeadings + heading;
  };

  if (sx == gx && sy == gy) {
    result.success = true;
    result.path.push_back(start_center);
    result.heading.push_back(0.0);
    result.length = 0.0;
    result.expansions = 1;
    return result;
  }

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(static_cast<std::size_t>(state_count), inf);
  std::vector<int> parent_state(static_cast<std::size_t>(state_count), -1);
  std::vector<int> parent_primitive(static_cast<std::size_t>(state_count), -1);
  std::vector<char> closed(static_cast<std::size_t>(state_count), 0);

  const Vec2 goal_center = grid.cellCenter(gx, gy);
  const auto heuristic = [&](int x, int y) {
    return distance(grid.cellCenter(x, y), goal_center);
  };

  struct OpenNode {
    double f = 0.0;
    double g = 0.0;
    int sequence = 0;
    int x = 0;
    int y = 0;
    int heading = 0;
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
  const int start_state = state_id(sx, sy, 0);
  g_score[static_cast<std::size_t>(start_state)] = 0.0;
  open.push(OpenNode{heuristic(sx, sy), 0.0, 0, sx, sy, 0});
  int sequence = 1;

  int goal_heading = -1;
  while (!open.empty()) {
    const OpenNode current = open.top();
    open.pop();
    const int current_state = state_id(current.x, current.y, current.heading);
    if (closed[static_cast<std::size_t>(current_state)]) {
      continue;
    }
    if (current.g != g_score[static_cast<std::size_t>(current_state)]) {
      continue;
    }
    closed[static_cast<std::size_t>(current_state)] = 1;
    ++result.expansions;

    if (current.x == gx && current.y == gy) {
      goal_heading = current.heading;
      break;
    }

    const Vec2 origin = grid.cellCenter(current.x, current.y);
    const double theta = static_cast<double>(current.heading) * (kPi * 0.5);
    for (int p = 0; p < static_cast<int>(primitives.size()); ++p) {
      int local_dx = 0;
      int local_dy = 0;
      rotateCell(primitives[static_cast<std::size_t>(p)].dx, primitives[static_cast<std::size_t>(p)].dy,
                 current.heading, local_dx, local_dy);
      const int nx = current.x + local_dx;
      const int ny = current.y + local_dy;
      if (!grid.inBounds(nx, ny) || grid.occupied(nx, ny)) {
        continue;
      }
      int next_heading = current.heading + primitives[static_cast<std::size_t>(p)].heading_delta;
      next_heading = (next_heading % kHeadings + kHeadings) % kHeadings;
      const int next_state = state_id(nx, ny, next_heading);
      if (closed[static_cast<std::size_t>(next_state)]) {
        continue;
      }
      if (!primitiveFree(grid, origin, theta, primitives[static_cast<std::size_t>(p)], radius,
                         params.body_length_m, params.body_width_m)) {
        continue;
      }
      const double tentative = current.g + primitives[static_cast<std::size_t>(p)].length;
      if (tentative + 1e-12 < g_score[static_cast<std::size_t>(next_state)]) {
        g_score[static_cast<std::size_t>(next_state)] = tentative;
        parent_state[static_cast<std::size_t>(next_state)] = current_state;
        parent_primitive[static_cast<std::size_t>(next_state)] = p;
        open.push(OpenNode{tentative + heuristic(nx, ny), tentative, sequence++, nx, ny, next_heading});
      }
    }
  }

  if (goal_heading < 0) {
    return result;
  }

  std::vector<int> states;
  for (int state = state_id(gx, gy, goal_heading); state >= 0;
       state = parent_state[static_cast<std::size_t>(state)]) {
    states.push_back(state);
  }
  std::reverse(states.begin(), states.end());

  result.path.push_back(start_center);
  result.heading.push_back(0.0);
  for (std::size_t i = 1; i < states.size(); ++i) {
    const int primitive_index = parent_primitive[static_cast<std::size_t>(states[i])];
    const int previous = states[i - 1];
    const int px = (previous / kHeadings) % width;
    const int py = (previous / kHeadings) / width;
    const int phe = previous % kHeadings;
    const Vec2 origin = grid.cellCenter(px, py);
    const double theta = static_cast<double>(phe) * (kPi * 0.5);
    const Primitive& primitive = primitives[static_cast<std::size_t>(primitive_index)];
    const double spacing = grid.resolution() * 0.25;
    const int steps = std::max(1, static_cast<int>(std::ceil(primitive.length / spacing)));
    for (int s = 1; s <= steps; ++s) {
      const double u = static_cast<double>(s) / static_cast<double>(steps);
      const Sample sample = interpolate(origin, theta, primitive, radius, u);
      result.path.push_back(Vec2{sample.x, sample.y});
      result.heading.push_back(sample.theta);
    }
    result.path.back() = grid.cellCenter((states[i] / kHeadings) % width, (states[i] / kHeadings) / width);
    const int end_heading = states[i] % kHeadings;
    result.heading.back() = static_cast<double>(end_heading) * (kPi * 0.5);
  }

  result.success = true;
  result.length = g_score[static_cast<std::size_t>(state_id(gx, gy, goal_heading))];
  return result;
}

}  // namespace planbench
