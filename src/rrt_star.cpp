#include "planbench/rrt_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace planbench {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Node {
  Vec2 point;
  int parent = -1;
  double cost = 0.0;
  std::vector<int> children;
};

double radiusFor(int node_count, double area, double step_size) {
  if (node_count <= 1) {
    return step_size;
  }
  const double gamma_star = 2.0 * std::sqrt(1.5) * std::sqrt(area / kPi);
  const double gamma = 1.5 * gamma_star;
  const double raw = gamma * std::sqrt(std::log(static_cast<double>(node_count)) / static_cast<double>(node_count));
  return std::clamp(raw, step_size, step_size * 12.0);
}

void reparent(std::vector<Node>& nodes, int node, int new_parent) {
  const int old_parent = nodes[static_cast<std::size_t>(node)].parent;
  if (old_parent >= 0) {
    auto& siblings = nodes[static_cast<std::size_t>(old_parent)].children;
    siblings.erase(std::remove(siblings.begin(), siblings.end(), node), siblings.end());
  }
  nodes[static_cast<std::size_t>(node)].parent = new_parent;
  nodes[static_cast<std::size_t>(new_parent)].children.push_back(node);
  nodes[static_cast<std::size_t>(node)].cost =
      nodes[static_cast<std::size_t>(new_parent)].cost +
      distance(nodes[static_cast<std::size_t>(new_parent)].point, nodes[static_cast<std::size_t>(node)].point);

  std::vector<int> queue = {node};
  for (std::size_t k = 0; k < queue.size(); ++k) {
    const int current = queue[k];
    for (int child : nodes[static_cast<std::size_t>(current)].children) {
      nodes[static_cast<std::size_t>(child)].cost =
          nodes[static_cast<std::size_t>(current)].cost +
          distance(nodes[static_cast<std::size_t>(current)].point, nodes[static_cast<std::size_t>(child)].point);
      queue.push_back(child);
    }
  }
}

struct GoalLink {
  bool success = false;
  int index = -1;
  double length = 0.0;
  std::vector<Vec2> path;
};

GoalLink bestGoalPath(const std::vector<Node>& nodes, const OccupancyGrid& grid, Vec2 goal, double goal_radius) {
  GoalLink best;
  double best_length = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const Vec2 point = nodes[static_cast<std::size_t>(i)].point;
    const double remaining = distance(point, goal);
    if (remaining > goal_radius) {
      continue;
    }
    if (remaining > 1e-12 && !grid.segmentFree(point, goal)) {
      continue;
    }
    const double length = nodes[static_cast<std::size_t>(i)].cost + remaining;
    if (length < best_length) {
      best_length = length;
      best.index = i;
    }
  }
  if (best.index < 0) {
    return best;
  }
  int walk = best.index;
  while (walk >= 0) {
    best.path.push_back(nodes[static_cast<std::size_t>(walk)].point);
    walk = nodes[static_cast<std::size_t>(walk)].parent;
  }
  std::reverse(best.path.begin(), best.path.end());
  if (distance(best.path.back(), goal) > 1e-9) {
    best.path.push_back(goal);
  }
  best.length = polylineLength(best.path);
  best.success = true;
  return best;
}

}  // namespace

RRTStarResult RRTStarPlanner::plan(const OccupancyGrid& grid, Vec2 start, Vec2 goal,
                                   const RRTStarParams& params) const {
  RRTStarResult result;
  if (!(params.step_size > 0.0) || !(params.goal_radius > 0.0) || params.max_iterations < 0) {
    return result;
  }
  if (!grid.pointFree(start.x, start.y) || !grid.pointFree(goal.x, goal.y)) {
    return result;
  }

  int horizon = params.max_iterations;
  std::vector<int> snapshots = params.snapshot_iterations;
  std::sort(snapshots.begin(), snapshots.end());
  snapshots.erase(std::unique(snapshots.begin(), snapshots.end()), snapshots.end());
  for (int snap : snapshots) {
    horizon = std::max(horizon, snap);
  }
  result.iterations = horizon;

  std::vector<Node> nodes;
  nodes.push_back(Node{start, -1, 0.0, {}});

  if (distance(start, goal) <= params.goal_radius &&
      (distance(start, goal) <= 1e-12 || grid.segmentFree(start, goal))) {
    GoalLink immediate = bestGoalPath(nodes, grid, goal, params.goal_radius);
    result.success = true;
    result.path = immediate.path;
    result.length = immediate.length;
    for (int snap : snapshots) {
      result.snapshots.push_back(RRTStarSnapshot{snap, true, immediate.length, 0.0});
    }
    return result;
  }

  std::mt19937 rng(params.seed);
  std::uniform_real_distribution<double> sample_x(0.0, grid.widthMeters());
  std::uniform_real_distribution<double> sample_y(0.0, grid.heightMeters());
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  const double area = grid.widthMeters() * grid.heightMeters();
  const auto started = std::chrono::steady_clock::now();
  std::size_t next_snapshot = 0;

  const auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  };

  const auto record_if_needed = [&](int iteration) {
    while (next_snapshot < snapshots.size() && snapshots[next_snapshot] == iteration) {
      GoalLink link = bestGoalPath(nodes, grid, goal, params.goal_radius);
      RRTStarSnapshot snap;
      snap.iterations = iteration;
      snap.success = link.success;
      snap.path_length = link.success ? link.length : 0.0;
      snap.runtime_ms = elapsed_ms();
      result.snapshots.push_back(snap);
      ++next_snapshot;
    }
  };

  for (int iteration = 1; iteration <= horizon; ++iteration) {
    Vec2 sample = goal;
    if (unit(rng) >= params.goal_sample_probability) {
      bool found = false;
      for (int attempt = 0; attempt < 32; ++attempt) {
        Vec2 candidate{sample_x(rng), sample_y(rng)};
        if (grid.pointFree(candidate.x, candidate.y)) {
          sample = candidate;
          found = true;
          break;
        }
      }
      if (!found) {
        record_if_needed(iteration);
        continue;
      }
    }

    int nearest = 0;
    double nearest_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
      const double dx = nodes[static_cast<std::size_t>(i)].point.x - sample.x;
      const double dy = nodes[static_cast<std::size_t>(i)].point.y - sample.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        nearest = i;
      }
    }

    const Vec2 from = nodes[static_cast<std::size_t>(nearest)].point;
    const double span = std::sqrt(nearest_d2);
    Vec2 steered = sample;
    if (span > params.step_size) {
      const double scale = params.step_size / span;
      steered = Vec2{from.x + (sample.x - from.x) * scale, from.y + (sample.y - from.y) * scale};
    }
    if (!grid.pointFree(steered.x, steered.y) || !grid.segmentFree(from, steered)) {
      record_if_needed(iteration);
      continue;
    }
    if (distance(from, steered) < 1e-9) {
      record_if_needed(iteration);
      continue;
    }

    const double radius = radiusFor(static_cast<int>(nodes.size()), area, params.step_size);
    const double radius_sq = radius * radius;
    std::vector<int> nearby;
    nearby.reserve(32);
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
      const double dx = nodes[static_cast<std::size_t>(i)].point.x - steered.x;
      const double dy = nodes[static_cast<std::size_t>(i)].point.y - steered.y;
      if (dx * dx + dy * dy <= radius_sq) {
        nearby.push_back(i);
      }
    }

    int parent = nearest;
    double best_cost = nodes[static_cast<std::size_t>(nearest)].cost + distance(from, steered);
    for (int candidate : nearby) {
      if (candidate == nearest) {
        continue;
      }
      const Vec2 candidate_point = nodes[static_cast<std::size_t>(candidate)].point;
      if (!grid.segmentFree(candidate_point, steered)) {
        continue;
      }
      const double cost = nodes[static_cast<std::size_t>(candidate)].cost + distance(candidate_point, steered);
      if (cost < best_cost) {
        best_cost = cost;
        parent = candidate;
      }
    }

    const int fresh = static_cast<int>(nodes.size());
    Node created;
    created.point = steered;
    created.parent = parent;
    created.cost = nodes[static_cast<std::size_t>(parent)].cost +
                   distance(nodes[static_cast<std::size_t>(parent)].point, steered);
    nodes.push_back(std::move(created));
    nodes[static_cast<std::size_t>(parent)].children.push_back(fresh);

    for (int candidate : nearby) {
      if (candidate == parent) {
        continue;
      }
      const double dx = nodes[static_cast<std::size_t>(candidate)].point.x - steered.x;
      const double dy = nodes[static_cast<std::size_t>(candidate)].point.y - steered.y;
      const double edge = std::sqrt(dx * dx + dy * dy);
      const double rewired = nodes[static_cast<std::size_t>(fresh)].cost + edge;
      if (rewired + 1e-12 < nodes[static_cast<std::size_t>(candidate)].cost &&
          grid.segmentFree(steered, nodes[static_cast<std::size_t>(candidate)].point)) {
        reparent(nodes, candidate, fresh);
      }
    }

    record_if_needed(iteration);
  }

  GoalLink link = bestGoalPath(nodes, grid, goal, params.goal_radius);
  result.success = link.success;
  result.path = std::move(link.path);
  result.length = link.success ? link.length : 0.0;
  return result;
}

}  // namespace planbench
