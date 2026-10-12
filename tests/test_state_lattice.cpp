#include "planbench/astar.hpp"
#include "planbench/state_lattice.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using planbench::AStarPlanner;
using planbench::LatticeParams;
using planbench::OccupancyGrid;
using planbench::StateLatticePlanner;
using planbench::Vec2;
using planbench::distance;

LatticeParams smallBody() {
  LatticeParams params;
  params.turning_radius_m = 2.0;
  params.body_length_m = 0.40;
  params.body_width_m = 0.30;
  return params;
}

void expectBodyFree(const OccupancyGrid& grid, const planbench::LatticeResult& result, const LatticeParams& params) {
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.path.size(), result.heading.size());
  ASSERT_GE(result.path.size(), 2u);
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    EXPECT_TRUE(grid.rectangleFree(result.path[i].x, result.path[i].y, result.heading[i], params.body_length_m,
                                   params.body_width_m));
  }
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    EXPECT_LT(distance(result.path[i - 1], result.path[i]), grid.resolution());
  }
}

}  // namespace

TEST(StateLattice, StraightLineOnAnEmptyGridIsOptimal) {
  OccupancyGrid grid(16, 8, 1.0);
  const auto params = smallBody();
  const Vec2 start = grid.cellCenter(2, 4);
  const Vec2 goal = grid.cellCenter(10, 4);
  const auto result = StateLatticePlanner{}.plan(grid, start, goal, params);
  expectBodyFree(grid, result, params);
  EXPECT_NEAR(result.length, 8.0, 1e-8);
  EXPECT_NEAR(distance(result.path.front(), start), 0.0, 1e-12);
  EXPECT_NEAR(distance(result.path.back(), goal), 0.0, 1e-12);
  EXPECT_GE(result.length + 1e-8, distance(start, goal));
}

TEST(StateLattice, SolidWallFails) {
  OccupancyGrid grid(16, 8, 1.0);
  for (int y = 0; y < grid.height(); ++y) {
    grid.setOccupied(7, y, true);
  }
  const auto result = StateLatticePlanner{}.plan(grid, grid.cellCenter(2, 4), grid.cellCenter(12, 4), smallBody());
  EXPECT_FALSE(result.success);
}

TEST(StateLattice, WideBodyCannotEnterANarrowCorridor) {
  OccupancyGrid grid(12, 5, 1.0);
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (y != 2) {
        grid.setOccupied(x, y, true);
      }
    }
  }
  const Vec2 start = grid.cellCenter(1, 2);
  const Vec2 goal = grid.cellCenter(10, 2);
  const auto astar = AStarPlanner{}.plan(grid, start, goal);
  ASSERT_TRUE(astar.success);
  EXPECT_NEAR(astar.length, 9.0, 1e-9);

  LatticeParams params;
  params.turning_radius_m = 2.0;
  params.body_length_m = 0.50;
  params.body_width_m = 1.20;
  const auto lattice = StateLatticePlanner{}.plan(grid, start, goal, params);
  EXPECT_FALSE(lattice.success);
}

TEST(StateLattice, DetourAroundABlockIsCollisionFree) {
  OccupancyGrid grid(24, 24, 0.5);
  for (int y = 8; y <= 16; ++y) {
    for (int x = 10; x <= 13; ++x) {
      grid.setOccupied(x, y, true);
    }
  }
  LatticeParams params;
  params.turning_radius_m = 1.0;
  params.body_length_m = 0.45;
  params.body_width_m = 0.30;
  const Vec2 start = grid.cellCenter(2, 12);
  const Vec2 goal = grid.cellCenter(21, 12);
  const auto result = StateLatticePlanner{}.plan(grid, start, goal, params);
  expectBodyFree(grid, result, params);
  EXPECT_GT(result.length, distance(start, goal) + 0.2);
  EXPECT_NEAR(result.turning_radius_m, 1.0, 1e-12);
}

TEST(StateLattice, StartEqualsGoal) {
  OccupancyGrid grid(8, 8, 1.0);
  const Vec2 pose = grid.cellCenter(3, 3);
  const auto result = StateLatticePlanner{}.plan(grid, pose, pose, smallBody());
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.path.size(), 1u);
  EXPECT_NEAR(result.length, 0.0, 1e-12);
}

TEST(StateLattice, TurningRadiusSnapsToTheGrid) {
  OccupancyGrid grid(12, 12, 0.25);
  LatticeParams params;
  params.turning_radius_m = 0.90;
  params.body_length_m = 0.40;
  params.body_width_m = 0.20;
  const auto result = StateLatticePlanner{}.plan(grid, grid.cellCenter(2, 6), grid.cellCenter(9, 6), params);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.turning_radius_m, 1.0, 1e-12);
  expectBodyFree(grid, result, params);
}
