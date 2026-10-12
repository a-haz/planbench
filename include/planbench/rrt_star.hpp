#pragma once

#include "planbench/occupancy_grid.hpp"
#include "planbench/types.hpp"

#include <cstdint>
#include <vector>

namespace planbench {

struct RRTStarParams {
  int max_iterations = 1000;
  double step_size = 0.5;
  double goal_radius = 0.5;
  double goal_sample_probability = 0.1;
  std::uint32_t seed = 1;
  // 1-based iteration counts at which the current best path is recorded.
  // The search runs through max(max_iterations, last snapshot).
  std::vector<int> snapshot_iterations;
};

struct RRTStarSnapshot {
  int iterations = 0;
  bool success = false;
  double path_length = 0.0;
  double runtime_ms = 0.0;
};

struct RRTStarResult {
  bool success = false;
  std::vector<Vec2> path;
  double length = 0.0;
  int iterations = 0;
  std::vector<RRTStarSnapshot> snapshots;
};

// Asymptotically optimal RRT* in the plane (Karaman and Frazzoli).
// Samples are uniform in the grid bounds. Edges are straight segments and are
// rejected unless OccupancyGrid::segmentFree accepts them. The search tree is
// rewired within a radius that shrinks as O(sqrt(log n / n)).
class RRTStarPlanner {
 public:
  RRTStarResult plan(const OccupancyGrid& grid, Vec2 start, Vec2 goal,
                     const RRTStarParams& params) const;
};

}  // namespace planbench
