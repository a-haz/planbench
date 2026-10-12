#include "planbench/astar.hpp"
#include "planbench/map_generator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace {

using planbench::AStarPlanner;
using planbench::OccupancyGrid;
using planbench::Vec2;
using planbench::distance;
using planbench::polylineLength;

constexpr double kSqrt2 = 1.4142135623730951;

double octile(int dx, int dy, double resolution) {
  const int diagonal_steps = std::min(std::abs(dx), std::abs(dy));
  const int cardinal_steps = std::abs(dx) + std::abs(dy) - 2 * diagonal_steps;
  return static_cast<double>(cardinal_steps) * resolution + static_cast<double>(diagonal_steps) * resolution * std::sqrt(2.0);
}

bool diagonalBlocked(const OccupancyGrid& grid, int x, int y, int nx, int ny) {
  const int dx = nx - x;
  const int dy = ny - y;
  if (dx == 0 || dy == 0) {
    return false;
  }
  return grid.occupied(x + dx, y) || grid.occupied(x, y + dy);
}

// Independent uniform-cost search with the same 8-connected neighbor rule.
double exhaustiveLength(const OccupancyGrid& grid, int sx, int sy, int gx, int gy) {
  const int width = grid.width();
  const double resolution = grid.resolution();
  const double diagonal = resolution * std::sqrt(2.0);
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> cost(static_cast<std::size_t>(width * grid.height()), inf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  const auto id = [width](int x, int y) { return y * width + x; };
  cost[static_cast<std::size_t>(id(sx, sy))] = 0.0;
  open.push({0.0, id(sx, sy)});
  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!open.empty()) {
    const Item item = open.top();
    open.pop();
    const int current = item.second;
    if (item.first != cost[static_cast<std::size_t>(current)]) {
      continue;
    }
    const int x = current % width;
    const int y = current / width;
    if (x == gx && y == gy) {
      return item.first;
    }
    for (int k = 0; k < 8; ++k) {
      const int nx = x + dxs[k];
      const int ny = y + dys[k];
      if (!grid.inBounds(nx, ny) || grid.occupied(nx, ny)) {
        continue;
      }
      if (dxs[k] != 0 && dys[k] != 0 && diagonalBlocked(grid, x, y, nx, ny)) {
        continue;
      }
      const double next = item.first + ((dxs[k] != 0 && dys[k] != 0) ? diagonal : resolution);
      const int nid = id(nx, ny);
      if (next + 1e-12 < cost[static_cast<std::size_t>(nid)]) {
        cost[static_cast<std::size_t>(nid)] = next;
        open.push({next, nid});
      }
    }
  }
  return inf;
}

}  // namespace

TEST(AStar, EmptyDiagonalIsOptimal) {
  OccupancyGrid grid(5, 5, 1.0);
  const auto result = AStarPlanner{}.plan(grid, grid.cellCenter(0, 0), grid.cellCenter(4, 4));
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.path.size(), 5u);
  EXPECT_NEAR(result.length, 4.0 * kSqrt2, 1e-9);
  EXPECT_NEAR(result.length, polylineLength(result.path), 1e-9);
}

TEST(AStar, EmptyOctileDistanceMatchesClosedForm) {
  OccupancyGrid grid(6, 4, 0.5);
  const auto result = AStarPlanner{}.plan(grid, grid.cellCenter(0, 0), grid.cellCenter(5, 2));
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.length, octile(5, 2, 0.5), 1e-9);
}

TEST(AStar, AllPairsOnEmptyGridMatchOctileDistance) {
  OccupancyGrid grid(5, 5, 1.0);
  AStarPlanner planner;
  for (int y0 = 0; y0 < grid.height(); ++y0) {
    for (int x0 = 0; x0 < grid.width(); ++x0) {
      for (int y1 = 0; y1 < grid.height(); ++y1) {
        for (int x1 = 0; x1 < grid.width(); ++x1) {
          const auto result = planner.plan(grid, grid.cellCenter(x0, y0), grid.cellCenter(x1, y1));
          ASSERT_TRUE(result.success);
          EXPECT_NEAR(result.length, octile(x1 - x0, y1 - y0, 1.0), 1e-9);
        }
      }
    }
  }
}

TEST(AStar, OpenCorridorIsTheStraightLine) {
  // A column wall with one gap on the start row. The straight line is feasible
  // and matches the octile lower bound, so it is the unique optimum.
  OccupancyGrid grid(7, 5, 1.0);
  for (int y = 0; y < grid.height(); ++y) {
    if (y != 2) {
      grid.setOccupied(3, y, true);
    }
  }
  const auto result = AStarPlanner{}.plan(grid, grid.cellCenter(0, 2), grid.cellCenter(6, 2));
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.length, 6.0, 1e-9);
  ASSERT_EQ(result.path.size(), 7u);
  for (int i = 0; i < 7; ++i) {
    EXPECT_NEAR(result.path[static_cast<std::size_t>(i)].x, 0.5 + static_cast<double>(i), 1e-12);
    EXPECT_NEAR(result.path[static_cast<std::size_t>(i)].y, 2.5, 1e-12);
  }
}

TEST(AStar, CornerCuttingIsRejected) {
  // S at (1,2), goal at (2,0). The diagonal step into (2,1) would clip the
  // occupied cell (1,1). The shortest legal path has length 3.
  //
  //   . S .
  //   # # .
  //   . . G
  OccupancyGrid grid(3, 3, 1.0);
  grid.setOccupied(0, 1, true);
  grid.setOccupied(1, 1, true);
  const auto result = AStarPlanner{}.plan(grid, grid.cellCenter(1, 2), grid.cellCenter(2, 0));
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.length, 3.0, 1e-9);
  for (const Vec2& point : result.path) {
    int ix = 0;
    int iy = 0;
    ASSERT_TRUE(grid.worldToCell(point.x, point.y, ix, iy));
    EXPECT_FALSE(grid.occupied(ix, iy));
  }
}

TEST(AStar, SolidWallHasNoPath) {
  OccupancyGrid grid(5, 5, 1.0);
  for (int y = 0; y < grid.height(); ++y) {
    grid.setOccupied(2, y, true);
  }
  const auto result = AStarPlanner{}.plan(grid, grid.cellCenter(0, 2), grid.cellCenter(4, 2));
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.path.empty());
}

TEST(AStar, OccupiedStartFails) {
  OccupancyGrid grid(4, 4, 1.0);
  grid.setOccupied(0, 0, true);
  const auto result = AStarPlanner{}.plan(grid, grid.cellCenter(0, 0), grid.cellCenter(3, 3));
  EXPECT_FALSE(result.success);
}

TEST(AStar, StartEqualsGoal) {
  OccupancyGrid grid(4, 4, 1.0);
  const Vec2 pose = grid.cellCenter(2, 1);
  const auto result = AStarPlanner{}.plan(grid, pose, pose);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.path.size(), 1u);
  EXPECT_NEAR(result.length, 0.0, 1e-12);
  EXPECT_NEAR(distance(result.path.front(), pose), 0.0, 1e-12);
}

TEST(AStar, MatchesExhaustiveSearchOnRandomMaps) {
  AStarPlanner planner;
  for (std::uint32_t seed = 1; seed <= 12; ++seed) {
    const auto map = planbench::generateMap(22, 18, 1.0, 0.20, seed, 1);
    int sx = 0;
    int sy = 0;
    int gx = 0;
    int gy = 0;
    ASSERT_TRUE(map.grid.worldToCell(map.start.x, map.start.y, sx, sy));
    ASSERT_TRUE(map.grid.worldToCell(map.goal.x, map.goal.y, gx, gy));
    const auto result = planner.plan(map.grid, map.start, map.goal);
    const double exhaustive = exhaustiveLength(map.grid, sx, sy, gx, gy);
    if (!std::isfinite(exhaustive)) {
      EXPECT_FALSE(result.success);
    } else {
      ASSERT_TRUE(result.success);
      EXPECT_NEAR(result.length, exhaustive, 1e-8);
      EXPECT_NEAR(result.length, polylineLength(result.path), 1e-8);
    }
  }
}

TEST(AStar, PathStepsAreLegal) {
  const auto map = planbench::generateMap(30, 24, 0.5, 0.18, 9, 2);
  const auto result = AStarPlanner{}.plan(map.grid, map.start, map.goal);
  ASSERT_TRUE(result.success);
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
    ASSERT_TRUE(map.grid.worldToCell(result.path[i - 1].x, result.path[i - 1].y, x0, y0));
    ASSERT_TRUE(map.grid.worldToCell(result.path[i].x, result.path[i].y, x1, y1));
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    EXPECT_LE(dx, 1);
    EXPECT_LE(dy, 1);
    EXPECT_FALSE(dx == 0 && dy == 0);
    EXPECT_FALSE(map.grid.occupied(x1, y1));
    if (dx == 1 && dy == 1) {
      EXPECT_FALSE(map.grid.occupied(x0 + (x1 - x0), y0));
      EXPECT_FALSE(map.grid.occupied(x0, y0 + (y1 - y0)));
    }
  }
}
