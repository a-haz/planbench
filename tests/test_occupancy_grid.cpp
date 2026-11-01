#include "planbench/occupancy_grid.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using planbench::OccupancyGrid;
using planbench::Vec2;

// All tests use a 4 x 3 grid with 0.5 m cells, so the map is 2.0 m wide and 1.5 m tall.
OccupancyGrid smallGrid() { return OccupancyGrid(4, 3, 0.5); }

}  // namespace

TEST(OccupancyGrid, SizeInMeters) {
  const OccupancyGrid grid = smallGrid();
  EXPECT_DOUBLE_EQ(grid.widthMeters(), 2.0);   // 4 * 0.5
  EXPECT_DOUBLE_EQ(grid.heightMeters(), 1.5);  // 3 * 0.5
}

TEST(OccupancyGrid, RejectsNonFiniteOrNegativeResolution) {
  EXPECT_THROW(OccupancyGrid(4, 3, -0.5), std::invalid_argument);
  EXPECT_THROW(OccupancyGrid(4, 3, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(OccupancyGrid(4, 3, std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(OccupancyGrid, InBoundsEdges) {
  const OccupancyGrid grid = smallGrid();
  // Valid indices are ix in [0, 3] and iy in [0, 2].
  EXPECT_TRUE(grid.inBounds(0, 0));
  EXPECT_TRUE(grid.inBounds(3, 2));
  EXPECT_FALSE(grid.inBounds(4, 0));
  EXPECT_FALSE(grid.inBounds(0, 3));
  EXPECT_FALSE(grid.inBounds(-1, 0));
  EXPECT_FALSE(grid.inBounds(0, -1));
}

TEST(OccupancyGrid, WorldToCellUsesFloorOfCoordinateOverResolution) {
  const OccupancyGrid grid = smallGrid();
  int ix = -1;
  int iy = -1;

  ASSERT_TRUE(grid.worldToCell(0.0, 0.0, ix, iy));
  EXPECT_EQ(ix, 0);
  EXPECT_EQ(iy, 0);

  // 0.49 / 0.5 = 0.98 and 0.99 / 0.5 = 1.98, so the cell is (0, 1).
  ASSERT_TRUE(grid.worldToCell(0.49, 0.99, ix, iy));
  EXPECT_EQ(ix, 0);
  EXPECT_EQ(iy, 1);

  // A point on a grid line belongs to the cell above it: 1.25 / 0.5 = 2.5 and 0.5 / 0.5 = 1.
  ASSERT_TRUE(grid.worldToCell(1.25, 0.5, ix, iy));
  EXPECT_EQ(ix, 2);
  EXPECT_EQ(iy, 1);

  // Just inside the far corner: 1.999 / 0.5 = 3.998 and 1.499 / 0.5 = 2.998, so (3, 2).
  ASSERT_TRUE(grid.worldToCell(1.999, 1.499, ix, iy));
  EXPECT_EQ(ix, 3);
  EXPECT_EQ(iy, 2);
}

TEST(OccupancyGrid, WorldToCellRejectsPointsOutsideTheMap) {
  const OccupancyGrid grid = smallGrid();
  int ix = 0;
  int iy = 0;
  // The map covers [0, 2.0) x [0, 1.5), so the far edges themselves are outside.
  EXPECT_FALSE(grid.worldToCell(2.0, 0.1, ix, iy));
  EXPECT_FALSE(grid.worldToCell(0.1, 1.5, ix, iy));
  EXPECT_FALSE(grid.worldToCell(-1e-9, 0.1, ix, iy));
  EXPECT_FALSE(grid.worldToCell(0.1, -1e-9, ix, iy));
  EXPECT_FALSE(grid.worldToCell(std::numeric_limits<double>::quiet_NaN(), 0.1, ix, iy));
}

TEST(OccupancyGrid, CellCenterIsHalfACellIn) {
  const OccupancyGrid grid = smallGrid();
  // Center of (ix, iy) is ((ix + 0.5) * 0.5, (iy + 0.5) * 0.5).
  const Vec2 first = grid.cellCenter(0, 0);
  EXPECT_DOUBLE_EQ(first.x, 0.25);
  EXPECT_DOUBLE_EQ(first.y, 0.25);
  const Vec2 last = grid.cellCenter(3, 2);
  EXPECT_DOUBLE_EQ(last.x, 1.75);
  EXPECT_DOUBLE_EQ(last.y, 1.25);
}

TEST(OccupancyGrid, CellCenterMapsBackToItsCell) {
  const OccupancyGrid grid = smallGrid();
  for (int iy = 0; iy < grid.height(); ++iy) {
    for (int ix = 0; ix < grid.width(); ++ix) {
      const Vec2 center = grid.cellCenter(ix, iy);
      int cx = -1;
      int cy = -1;
      ASSERT_TRUE(grid.worldToCell(center.x, center.y, cx, cy));
      EXPECT_EQ(cx, ix);
      EXPECT_EQ(cy, iy);
    }
  }
}

TEST(OccupancyGrid, OccupiedLookupTouchesOnlyTheSetCell) {
  OccupancyGrid grid = smallGrid();
  grid.setOccupied(2, 1, true);
  for (int iy = 0; iy < grid.height(); ++iy) {
    for (int ix = 0; ix < grid.width(); ++ix) {
      EXPECT_EQ(grid.occupied(ix, iy), ix == 2 && iy == 1) << ix << "," << iy;
    }
  }
  // (1, 2) is the transpose of the set cell and must stay free.
  EXPECT_FALSE(grid.occupied(1, 2));

  grid.setOccupied(2, 1, false);
  EXPECT_FALSE(grid.occupied(2, 1));
}

TEST(OccupancyGrid, OutsideCellsReadAsOccupied) {
  const OccupancyGrid grid = smallGrid();
  EXPECT_TRUE(grid.occupied(4, 0));
  EXPECT_TRUE(grid.occupied(0, 3));
  EXPECT_TRUE(grid.occupied(-1, 0));
  EXPECT_TRUE(grid.occupied(0, -1));
}

TEST(OccupancyGrid, OutsideWritesThrow) {
  OccupancyGrid grid = smallGrid();
  EXPECT_THROW(grid.setOccupied(0, 3, true), std::out_of_range);
  EXPECT_THROW(grid.setOccupied(-1, 0, true), std::out_of_range);
  EXPECT_THROW(grid.cellCenter(4, 0), std::out_of_range);
}
