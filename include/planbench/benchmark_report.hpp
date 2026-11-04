#pragma once

#include "planbench/occupancy_grid.hpp"
#include "planbench/rrt_star.hpp"
#include "planbench/state_lattice.hpp"
#include "planbench/types.hpp"

#include <string>

namespace planbench::benchmark {

// Totals for one planner over a set of trials. Lengths are summed only for
// successful trials, runtimes for every trial.
struct Stats {
  int trials = 0;
  int successes = 0;
  double length_sum = 0.0;
  double runtime_sum = 0.0;
};

void addTrial(Stats& stats, bool success, double length_m, double runtime_ms);

std::string formatDouble(double value, int digits);

// One CSV line: table,planner,density,iterations,trials,successes,
// success_rate,mean_path_length_m,mean_runtime_ms. The mean length is left
// empty when there were no successes.
std::string csvRow(const std::string& table, const std::string& planner, double density, int iterations,
                   const Stats& stats);

// Checks that a planner result is consistent with the map before it is counted.
bool validAStar(const OccupancyGrid& grid, Vec2 start, Vec2 goal, const PlanResult& result);
bool validRRT(const OccupancyGrid& grid, Vec2 start, Vec2 goal, const RRTStarResult& result);
bool validLattice(const OccupancyGrid& grid, Vec2 start, Vec2 goal, const LatticeResult& result,
                  const LatticeParams& params);

}  // namespace planbench::benchmark
