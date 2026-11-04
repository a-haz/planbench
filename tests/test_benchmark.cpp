#include "planbench/astar.hpp"
#include "planbench/benchmark_report.hpp"
#include "planbench/occupancy_grid.hpp"

#include <gtest/gtest.h>

namespace {

using planbench::OccupancyGrid;
using planbench::PlanResult;
using planbench::RRTStarResult;
using planbench::Vec2;
using planbench::benchmark::addTrial;
using planbench::benchmark::csvRow;
using planbench::benchmark::Stats;
using planbench::benchmark::validAStar;
using planbench::benchmark::validRRT;

// 4 x 3 grid with 0.5 m cells and a wall in the middle row:
//
//   y=2  .  .  .  .
//   y=1  S  #  #  G
//   y=0  .  .  .  .
//
// S is cell (0, 1) with center (0.25, 0.75) and G is cell (3, 1) with center (1.75, 0.75).
// Diagonals that cut a wall corner are not allowed, so the shortest grid path goes
// round the wall: 1 step up or down, 3 steps across, 1 step back = 5 * 0.5 = 2.5 m,
// visiting 6 cells.
OccupancyGrid wallGrid() {
  OccupancyGrid grid(4, 3, 0.5);
  grid.setOccupied(1, 1, true);
  grid.setOccupied(2, 1, true);
  return grid;
}

const Vec2 kStart{0.25, 0.75};
const Vec2 kGoal{1.75, 0.75};

}  // namespace

TEST(Benchmark, AddTrialCountsSuccessesAndTimesEveryTrial) {
  Stats stats;
  addTrial(stats, true, 2.5, 1.0);
  addTrial(stats, false, 99.0, 3.0);  // failed trial: its length must not be added
  addTrial(stats, true, 3.5, 2.0);

  EXPECT_EQ(stats.trials, 3);
  EXPECT_EQ(stats.successes, 2);
  EXPECT_DOUBLE_EQ(stats.length_sum, 6.0);   // 2.5 + 3.5
  EXPECT_DOUBLE_EQ(stats.runtime_sum, 6.0);  // 1.0 + 3.0 + 2.0
}

TEST(Benchmark, CsvRowAveragesLengthOverSuccessesAndRuntimeOverTrials) {
  Stats stats;
  addTrial(stats, true, 2.5, 1.0);
  addTrial(stats, false, 99.0, 3.0);
  addTrial(stats, true, 3.5, 2.0);

  // success rate 2 / 3, mean length 6.0 / 2 = 3.0, mean runtime 6.0 / 3 = 2.0
  EXPECT_EQ(csvRow("summary", "astar", 0.1, 0, stats), "summary,astar,0.10,0,3,2,0.666667,3.000000,2.000000");
}

TEST(Benchmark, CsvRowLeavesLengthEmptyWithoutSuccesses) {
  Stats stats;
  addTrial(stats, false, 0.0, 1.0);
  addTrial(stats, false, 0.0, 2.0);

  EXPECT_EQ(csvRow("rrt_budget", "rrt_star", 0.2, 40, stats), "rrt_budget,rrt_star,0.20,40,2,0,0.000000,,1.500000");
}

TEST(Benchmark, AStarOnWallGridIsValidAndSummarized) {
  const OccupancyGrid grid = wallGrid();
  const PlanResult result = planbench::AStarPlanner{}.plan(grid, kStart, kGoal);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.length, 2.5, 1e-9);
  EXPECT_EQ(result.path.size(), 6u);
  EXPECT_TRUE(validAStar(grid, kStart, kGoal, result));

  Stats stats;
  addTrial(stats, result.success, result.length, 0.25);
  EXPECT_EQ(csvRow("summary", "astar", 0.1, 0, stats), "summary,astar,0.10,0,1,1,1.000000,2.500000,0.250000");
}

TEST(Benchmark, ValidAStarRejectsWrongLength) {
  const OccupancyGrid grid = wallGrid();
  PlanResult result = planbench::AStarPlanner{}.plan(grid, kStart, kGoal);
  ASSERT_TRUE(result.success);

  result.length = 2.0;  // the path itself is still 2.5 m long
  EXPECT_FALSE(validAStar(grid, kStart, kGoal, result));
}

TEST(Benchmark, ValidAStarRejectsPathThroughWall) {
  const OccupancyGrid grid = wallGrid();
  PlanResult result;
  result.success = true;
  // Straight along y=1, through the two wall cells: 3 * 0.5 = 1.5 m.
  result.path = {grid.cellCenter(0, 1), grid.cellCenter(1, 1), grid.cellCenter(2, 1), grid.cellCenter(3, 1)};
  result.length = 1.5;
  EXPECT_FALSE(validAStar(grid, kStart, kGoal, result));
}

TEST(Benchmark, ValidAStarFailureMustHaveEmptyPath) {
  OccupancyGrid grid = wallGrid();
  grid.setOccupied(1, 0, true);
  grid.setOccupied(1, 2, true);  // column x=1 is now fully blocked

  PlanResult result = planbench::AStarPlanner{}.plan(grid, kStart, kGoal);
  ASSERT_FALSE(result.success);
  EXPECT_TRUE(validAStar(grid, kStart, kGoal, result));

  result.path = {kStart};
  EXPECT_FALSE(validAStar(grid, kStart, kGoal, result));
}

TEST(Benchmark, ValidRRTChecksEndpointsLengthAndWalls) {
  const OccupancyGrid grid = wallGrid();
  RRTStarResult around;
  around.success = true;
  // Up to the top row, across, and back down: 0.5 + 1.5 + 0.5 = 2.5 m.
  around.path = {kStart, {0.25, 1.25}, {1.75, 1.25}, kGoal};
  around.length = 2.5;
  EXPECT_TRUE(validRRT(grid, kStart, kGoal, around));

  RRTStarResult through;
  through.success = true;
  through.path = {kStart, kGoal};  // 1.5 m straight through the wall
  through.length = 1.5;
  EXPECT_FALSE(validRRT(grid, kStart, kGoal, through));

  RRTStarResult short_of_goal = around;
  short_of_goal.path.back() = {1.75, 1.25};
  short_of_goal.length = 2.0;  // matches the shortened polyline, but it ends 0.5 m from the goal
  EXPECT_FALSE(validRRT(grid, kStart, kGoal, short_of_goal));
}
