#pragma once

#include "planbench/occupancy_grid.hpp"
#include "planbench/types.hpp"

#include <cstdint>

namespace planbench {

struct GeneratedMap {
  OccupancyGrid grid;
  Vec2 start;
  Vec2 goal;
};

// Synthetic Bernoulli occupancy map.
// Cells are visited in row-major order (y, then x) and set occupied with
// probability `occupancy_probability` from std::mt19937(seed). A square
// Chebyshev window of `clearance_cells` around the start and goal cells is
// then forced free. Start is cell (clearance, height/2) and goal is cell
// (width-1-clearance, height/2), both reported as cell centers.
// The same arguments always rebuild the same map.
GeneratedMap generateMap(int width, int height, double resolution,
                         double occupancy_probability, std::uint32_t seed,
                         int clearance_cells);

}  // namespace planbench
