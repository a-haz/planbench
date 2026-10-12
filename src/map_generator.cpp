#include "planbench/map_generator.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace planbench {

GeneratedMap generateMap(int width, int height, double resolution, double occupancy_probability,
                         std::uint32_t seed, int clearance_cells) {
  if (occupancy_probability < 0.0 || occupancy_probability > 1.0) {
    throw std::invalid_argument("occupancy probability must lie in [0, 1]");
  }
  if (clearance_cells < 0) {
    throw std::invalid_argument("clearance must be non-negative");
  }

  OccupancyGrid grid(width, height, resolution);
  std::mt19937 rng(seed);
  std::bernoulli_distribution occupied(occupancy_probability);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      grid.setOccupied(x, y, occupied(rng));
    }
  }

  int start_x = clearance_cells;
  int goal_x = width - 1 - clearance_cells;
  if (goal_x <= start_x) {
    start_x = 0;
    goal_x = width - 1;
  }
  const int start_y = std::min(height - 1, std::max(0, height / 2));
  const int goal_y = start_y;

  const auto clear_window = [&](int cx, int cy) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (std::max(std::abs(x - cx), std::abs(y - cy)) <= clearance_cells) {
          grid.setOccupied(x, y, false);
        }
      }
    }
  };
  clear_window(start_x, start_y);
  clear_window(goal_x, goal_y);

  GeneratedMap map{std::move(grid), {}, {}};
  map.start = map.grid.cellCenter(start_x, start_y);
  map.goal = map.grid.cellCenter(goal_x, goal_y);
  return map;
}

}  // namespace planbench
