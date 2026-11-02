#pragma once

#include "planbench/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace planbench {

// Axis-aligned occupancy grid. Cell (ix, iy) covers
// [ix * resolution, (ix + 1) * resolution) x [iy * resolution, (iy + 1) * resolution).
// A cell value of 1 is occupied. Coordinates outside the grid are not free.
class OccupancyGrid {
 public:
  OccupancyGrid(int width, int height, double resolution);

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double widthMeters() const { return static_cast<double>(width_) * resolution_; }
  double heightMeters() const { return static_cast<double>(height_) * resolution_; }

  bool inBounds(int ix, int iy) const;
  bool occupied(int ix, int iy) const;
  void setOccupied(int ix, int iy, bool value);

  bool worldToCell(double x, double y, int& ix, int& iy) const;
  Vec2 cellCenter(int ix, int iy) const;

  bool pointFree(double x, double y) const;

  // True when the closed segment stays inside the map and does not touch an
  // occupied cell. A segment that passes through a grid corner also touches
  // the two side cells, so diagonal moves cannot clip an occupied corner.
  // A point on a grid line belongs to the cell above or right of it, as in
  // worldToCell, so a segment lying exactly on a grid line is only checked
  // against the cells on that side.
  bool segmentFree(Vec2 a, Vec2 b) const;

  // Oriented rectangle: center (x, y), yaw theta, length along heading, width
  // across heading. Touching an occupied cell or leaving the map is a collision,
  // and so is a pose with a non-finite coordinate or heading.
  bool rectangleFree(double x, double y, double theta, double length, double width) const;

 private:
  int index(int ix, int iy) const { return iy * width_ + ix; }

  int width_ = 0;
  int height_ = 0;
  double resolution_ = 1.0;
  std::vector<std::uint8_t> cells_;
};

}  // namespace planbench
