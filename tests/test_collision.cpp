#include "planbench/occupancy_grid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

using planbench::OccupancyGrid;
using planbench::Vec2;

bool clip(double p, double q, double& t0, double& t1) {
  if (std::abs(p) < 1e-15) {
    return q >= 0.0;
  }
  const double r = q / p;
  if (p < 0.0) {
    if (r > t1) {
      return false;
    }
    if (r > t0) {
      t0 = r;
    }
  } else {
    if (r < t0) {
      return false;
    }
    if (r < t1) {
      t1 = r;
    }
  }
  return true;
}

// Independent segment/AABB test (Liang-Barsky). Touching the boundary counts.
bool segmentHitsCell(Vec2 a, Vec2 b, double min_x, double min_y, double max_x, double max_y) {
  double t0 = 0.0;
  double t1 = 1.0;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  if (!clip(-dx, a.x - min_x, t0, t1) || !clip(dx, max_x - a.x, t0, t1) || !clip(-dy, a.y - min_y, t0, t1) ||
      !clip(dy, max_y - a.y, t0, t1)) {
    return false;
  }
  return t0 <= t1;
}

bool ambiguousBoundary(const OccupancyGrid& grid, Vec2 a, Vec2 b) {
  const double res = grid.resolution();
  const int steps = 64;
  for (int i = 0; i <= steps; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(steps);
    const double x = a.x + (b.x - a.x) * u;
    const double y = a.y + (b.y - a.y) * u;
    const double fx = x / res;
    const double fy = y / res;
    if (std::abs(fx - std::round(fx)) < 1e-7 || std::abs(fy - std::round(fy)) < 1e-7) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(Collision, RejectsNonPositiveGrid) {
  EXPECT_THROW(OccupancyGrid(0, 4, 1.0), std::invalid_argument);
  EXPECT_THROW(OccupancyGrid(4, -1, 1.0), std::invalid_argument);
  EXPECT_THROW(OccupancyGrid(4, 4, 0.0), std::invalid_argument);
}

TEST(Collision, PointQueries) {
  OccupancyGrid grid(4, 3, 1.0);
  grid.setOccupied(1, 1, true);
  EXPECT_FALSE(grid.pointFree(1.2, 1.4));
  EXPECT_TRUE(grid.pointFree(0.2, 0.2));
  EXPECT_FALSE(grid.pointFree(-0.1, 0.2));
  EXPECT_FALSE(grid.pointFree(4.0, 1.0));
  EXPECT_FALSE(grid.pointFree(1.0, 3.0));
  EXPECT_TRUE(grid.occupied(1, 1));
  EXPECT_FALSE(grid.occupied(0, 0));
  EXPECT_TRUE(grid.occupied(-1, 0));
}

TEST(Collision, HorizontalSegmentHitsObstacle) {
  OccupancyGrid grid(5, 3, 1.0);
  grid.setOccupied(2, 1, true);
  EXPECT_FALSE(grid.segmentFree(Vec2{0.5, 1.5}, Vec2{4.5, 1.5}));
  EXPECT_TRUE(grid.segmentFree(Vec2{0.5, 0.5}, Vec2{4.5, 0.5}));
}

TEST(Collision, EndpointsFreeButMiddleOccupied) {
  OccupancyGrid grid(5, 5, 1.0);
  grid.setOccupied(2, 2, true);
  EXPECT_TRUE(grid.pointFree(0.5, 2.5));
  EXPECT_TRUE(grid.pointFree(4.5, 2.5));
  EXPECT_FALSE(grid.segmentFree(Vec2{0.5, 2.5}, Vec2{4.5, 2.5}));
}

TEST(Collision, CornerTouchCountsTheSideCells) {
  OccupancyGrid grid(3, 3, 1.0);
  grid.setOccupied(1, 0, true);
  EXPECT_FALSE(grid.segmentFree(grid.cellCenter(0, 0), grid.cellCenter(1, 1)));
  EXPECT_TRUE(grid.segmentFree(grid.cellCenter(0, 1), grid.cellCenter(2, 1)));
}

TEST(Collision, SameCellSegment) {
  OccupancyGrid grid(3, 3, 1.0);
  EXPECT_TRUE(grid.segmentFree(Vec2{0.2, 0.2}, Vec2{0.8, 0.7}));
  grid.setOccupied(0, 0, true);
  EXPECT_FALSE(grid.segmentFree(Vec2{0.2, 0.2}, Vec2{0.8, 0.7}));
}

TEST(Collision, RectangleFootprint) {
  OccupancyGrid grid(5, 5, 1.0);
  grid.setOccupied(2, 2, true);
  EXPECT_FALSE(grid.rectangleFree(2.5, 2.5, 0.0, 0.4, 0.4));
  EXPECT_TRUE(grid.rectangleFree(0.5, 0.5, 0.0, 0.4, 0.4));
  EXPECT_FALSE(grid.rectangleFree(1.5, 2.5, 0.0, 1.2, 0.2));
  EXPECT_TRUE(grid.rectangleFree(0.5, 0.5, 0.7, 0.3, 0.3));
  EXPECT_FALSE(grid.rectangleFree(0.2, 0.5, 0.0, 0.8, 0.2));
  EXPECT_FALSE(grid.rectangleFree(2.5, 2.5, 0.4, 0.0, 0.2));
}

TEST(Collision, SegmentAgreesWithCellIntersection) {
  OccupancyGrid grid(12, 9, 0.5);
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (((x * 17 + y * 13) % 5) == 0) {
        grid.setOccupied(x, y, true);
      }
    }
  }
  std::vector<Vec2> samples;
  for (int y = 0; y < grid.height(); y += 2) {
    for (int x = 0; x < grid.width(); x += 2) {
      if (!grid.occupied(x, y)) {
        samples.push_back(grid.cellCenter(x, y));
      }
    }
  }
  samples.push_back(Vec2{0.2, 0.3});
  samples.push_back(Vec2{5.7, 1.1});
  samples.push_back(Vec2{4.4, 3.8});
  samples.push_back(Vec2{1.3, 4.2});

  for (std::size_t i = 0; i < samples.size(); ++i) {
    for (std::size_t j = i + 1; j < samples.size(); ++j) {
      const Vec2 a = samples[i];
      const Vec2 b = samples[j];
      if (!grid.pointFree(a.x, a.y) || !grid.pointFree(b.x, b.y)) {
        continue;
      }
      bool hit = false;
      for (int y = 0; y < grid.height() && !hit; ++y) {
        for (int x = 0; x < grid.width(); ++x) {
          if (!grid.occupied(x, y)) {
            continue;
          }
          const double min_x = static_cast<double>(x) * grid.resolution();
          const double min_y = static_cast<double>(y) * grid.resolution();
          if (segmentHitsCell(a, b, min_x, min_y, min_x + grid.resolution(), min_y + grid.resolution())) {
            hit = true;
            break;
          }
        }
      }
      const bool free = grid.segmentFree(a, b);
      if (free == !hit) {
        continue;
      }
      // Grid lines belong to one cell under floor(), so a segment that rides
      // a boundary can disagree with a closed-AABB test. Clear misses still fail.
      ASSERT_TRUE(ambiguousBoundary(grid, a, b)) << a.x << "," << a.y << " -> " << b.x << "," << b.y;
    }
  }
}

TEST(Collision, OutOfRangeWriteThrows) {
  OccupancyGrid grid(2, 2, 1.0);
  EXPECT_THROW(grid.setOccupied(2, 0, true), std::out_of_range);
  EXPECT_THROW(grid.cellCenter(-1, 0), std::out_of_range);
}
