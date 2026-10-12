#pragma once

#include "planbench/occupancy_grid.hpp"
#include "planbench/types.hpp"

namespace planbench {

// 8-connected grid A* on cell centers.
// Cardinal steps cost `resolution`, diagonal steps cost `resolution * sqrt(2)`.
// Diagonal steps are rejected when either adjacent cardinal cell is occupied.
// The heuristic is the octile distance, which is consistent on this graph,
// so the returned path is optimal in grid length when one exists.
class AStarPlanner {
 public:
  PlanResult plan(const OccupancyGrid& grid, Vec2 start, Vec2 goal) const;
};

}  // namespace planbench
