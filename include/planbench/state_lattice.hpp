#pragma once

#include "planbench/occupancy_grid.hpp"
#include "planbench/types.hpp"

#include <vector>

namespace planbench {

struct LatticeParams {
  // Snapped to an integer number of cells so 90 degree arcs land on cell centers.
  double turning_radius_m = 1.0;
  double body_length_m = 0.70;
  double body_width_m = 0.40;
};

struct LatticeResult {
  bool success = false;
  std::vector<Vec2> path;
  std::vector<double> heading;
  double length = 0.0;
  int expansions = 0;
  double turning_radius_m = 0.0;
  double body_length_m = 0.0;
  double body_width_m = 0.0;
};

// State lattice over (cell x, cell y, 4 headings) with car-like primitives:
// forward straights and 90 degree circular arcs at a fixed turning radius.
// A rectangular body is collision-checked along each primitive.
// Search is A* with Euclidean distance as a consistent heuristic, so the
// result is optimal in arc length over this primitive set.
// The vehicle starts facing +x.
class StateLatticePlanner {
 public:
  LatticeResult plan(const OccupancyGrid& grid, Vec2 start, Vec2 goal,
                     const LatticeParams& params) const;
};

}  // namespace planbench
