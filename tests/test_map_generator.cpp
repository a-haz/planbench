#include "planbench/map_generator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace {

int occupiedCount(const planbench::OccupancyGrid& grid) {
  int count = 0;
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (grid.occupied(x, y)) {
        ++count;
      }
    }
  }
  return count;
}

bool sameCells(const planbench::OccupancyGrid& a, const planbench::OccupancyGrid& b) {
  if (a.width() != b.width() || a.height() != b.height()) {
    return false;
  }
  for (int y = 0; y < a.height(); ++y) {
    for (int x = 0; x < a.width(); ++x) {
      if (a.occupied(x, y) != b.occupied(x, y)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

TEST(MapGenerator, SameSeedRebuildsTheSameMap) {
  const auto first = planbench::generateMap(40, 28, 0.25, 0.27, 1234, 2);
  const auto second = planbench::generateMap(40, 28, 0.25, 0.27, 1234, 2);
  EXPECT_TRUE(sameCells(first.grid, second.grid));
  EXPECT_NEAR(first.start.x, second.start.x, 0.0);
  EXPECT_NEAR(first.goal.y, second.goal.y, 0.0);
}

TEST(MapGenerator, DifferentSeedsDiffer) {
  const auto first = planbench::generateMap(40, 40, 1.0, 0.30, 1, 1);
  const auto second = planbench::generateMap(40, 40, 1.0, 0.30, 2, 1);
  EXPECT_FALSE(sameCells(first.grid, second.grid));
}

TEST(MapGenerator, ClearanceWindowIsFreeEvenWhenEveryCellStartsOccupied) {
  const auto map = planbench::generateMap(21, 15, 0.5, 1.0, 7, 2);
  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  ASSERT_TRUE(map.grid.worldToCell(map.start.x, map.start.y, sx, sy));
  ASSERT_TRUE(map.grid.worldToCell(map.goal.x, map.goal.y, gx, gy));
  EXPECT_EQ(sx, 2);
  EXPECT_EQ(gx, 18);
  for (int y = 0; y < map.grid.height(); ++y) {
    for (int x = 0; x < map.grid.width(); ++x) {
      const int ds = std::max(std::abs(x - sx), std::abs(y - sy));
      const int dg = std::max(std::abs(x - gx), std::abs(y - gy));
      if (ds <= 2 || dg <= 2) {
        EXPECT_FALSE(map.grid.occupied(x, y));
      } else {
        EXPECT_TRUE(map.grid.occupied(x, y));
      }
    }
  }
  EXPECT_TRUE(map.grid.pointFree(map.start.x, map.start.y));
  EXPECT_TRUE(map.grid.pointFree(map.goal.x, map.goal.y));
}

TEST(MapGenerator, DensityTracksTheRequestedProbability) {
  const auto map = planbench::generateMap(80, 80, 0.2, 0.30, 42, 1);
  const double fraction = static_cast<double>(occupiedCount(map.grid)) / (80.0 * 80.0);
  EXPECT_GT(fraction, 0.25);
  EXPECT_LT(fraction, 0.35);
}

TEST(MapGenerator, RejectsBadArguments) {
  EXPECT_THROW(planbench::generateMap(8, 8, 1.0, 1.2, 1, 1), std::invalid_argument);
  EXPECT_THROW(planbench::generateMap(8, 8, 1.0, -0.1, 1, 1), std::invalid_argument);
  EXPECT_THROW(planbench::generateMap(8, 8, 1.0, 0.1, 1, -1), std::invalid_argument);
}
