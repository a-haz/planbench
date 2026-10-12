#include "planbench/rrt_star.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using planbench::OccupancyGrid;
using planbench::RRTStarParams;
using planbench::RRTStarPlanner;
using planbench::Vec2;
using planbench::distance;
using planbench::polylineLength;

RRTStarParams baseParams() {
  RRTStarParams params;
  params.max_iterations = 500;
  params.step_size = 0.7;
  params.goal_radius = 0.7;
  params.goal_sample_probability = 0.15;
  params.seed = 11;
  return params;
}

void expectValid(const OccupancyGrid& grid, Vec2 start, Vec2 goal, const planbench::RRTStarResult& result) {
  ASSERT_TRUE(result.success);
  ASSERT_GE(result.path.size(), 2u);
  EXPECT_NEAR(distance(result.path.front(), start), 0.0, 1e-9);
  EXPECT_NEAR(distance(result.path.back(), goal), 0.0, 1e-9);
  EXPECT_NEAR(result.length, polylineLength(result.path), 1e-8);
  EXPECT_GE(result.length + 1e-8, distance(start, goal));
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    EXPECT_TRUE(grid.segmentFree(result.path[i - 1], result.path[i]));
    EXPECT_TRUE(grid.pointFree(result.path[i].x, result.path[i].y));
  }
}

}  // namespace

TEST(RRTStar, EmptyMapPathIsValid) {
  OccupancyGrid grid(16, 16, 0.5);
  const Vec2 start = grid.cellCenter(1, 1);
  const Vec2 goal = grid.cellCenter(14, 14);
  const auto result = RRTStarPlanner{}.plan(grid, start, goal, baseParams());
  expectValid(grid, start, goal, result);
}

TEST(RRTStar, DeterministicForAFixedSeed) {
  OccupancyGrid grid(14, 12, 0.5);
  grid.setOccupied(6, 4, true);
  grid.setOccupied(6, 5, true);
  grid.setOccupied(7, 5, true);
  grid.setOccupied(8, 6, true);
  const Vec2 start = grid.cellCenter(1, 2);
  const Vec2 goal = grid.cellCenter(12, 9);
  RRTStarPlanner planner;
  const auto params = baseParams();
  const auto first = planner.plan(grid, start, goal, params);
  const auto second = planner.plan(grid, start, goal, params);
  ASSERT_EQ(first.success, second.success);
  ASSERT_EQ(first.path.size(), second.path.size());
  for (std::size_t i = 0; i < first.path.size(); ++i) {
    EXPECT_NEAR(first.path[i].x, second.path[i].x, 0.0);
    EXPECT_NEAR(first.path[i].y, second.path[i].y, 0.0);
  }
  if (first.success) {
    expectValid(grid, start, goal, first);
  }
}

TEST(RRTStar, SolidWallFails) {
  OccupancyGrid grid(12, 8, 1.0);
  for (int y = 0; y < grid.height(); ++y) {
    grid.setOccupied(6, y, true);
  }
  auto params = baseParams();
  params.max_iterations = 200;
  const auto result = RRTStarPlanner{}.plan(grid, grid.cellCenter(1, 3), grid.cellCenter(10, 3), params);
  EXPECT_FALSE(result.success);
}

TEST(RRTStar, OccupiedGoalFails) {
  OccupancyGrid grid(8, 8, 1.0);
  grid.setOccupied(6, 6, true);
  const auto result = RRTStarPlanner{}.plan(grid, grid.cellCenter(1, 1), grid.cellCenter(6, 6), baseParams());
  EXPECT_FALSE(result.success);
}

TEST(RRTStar, LengthDoesNotGrowAsTheIterationBudgetGrows) {
  OccupancyGrid grid(18, 14, 0.5);
  for (int y = 3; y <= 10; ++y) {
    grid.setOccupied(8, y, true);
  }
  grid.setOccupied(8, 6, false);
  const Vec2 start = grid.cellCenter(2, 7);
  const Vec2 goal = grid.cellCenter(15, 7);
  auto params = baseParams();
  params.max_iterations = 800;
  params.snapshot_iterations = {100, 200, 400, 800};
  params.seed = 23;
  const auto result = RRTStarPlanner{}.plan(grid, start, goal, params);
  ASSERT_EQ(result.snapshots.size(), 4u);
  bool seen = false;
  double best = 0.0;
  for (const auto& snap : result.snapshots) {
    if (!snap.success) {
      EXPECT_FALSE(seen);
      continue;
    }
    if (!seen) {
      best = snap.path_length;
      seen = true;
    } else {
      EXPECT_LE(snap.path_length, best + 1e-8);
      best = snap.path_length;
    }
    EXPECT_GE(snap.path_length + 1e-8, distance(start, goal));
    EXPECT_GT(snap.runtime_ms, 0.0);
  }
  ASSERT_TRUE(result.success);
  expectValid(grid, start, goal, result);
  EXPECT_NEAR(result.length, result.snapshots.back().path_length, 1e-8);
}

TEST(RRTStar, ScatteredObstaclesStayCollisionFree) {
  OccupancyGrid grid(20, 16, 0.4);
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if ((x + 3 * y) % 11 == 0 && x > 2 && x < grid.width() - 2) {
        grid.setOccupied(x, y, true);
      }
    }
  }
  const Vec2 start = grid.cellCenter(1, 1);
  const Vec2 goal = grid.cellCenter(18, 14);
  auto params = baseParams();
  params.max_iterations = 700;
  params.step_size = 0.55;
  params.goal_radius = 0.55;
  params.seed = 5;
  const auto result = RRTStarPlanner{}.plan(grid, start, goal, params);
  ASSERT_TRUE(result.success);
  expectValid(grid, start, goal, result);
}
