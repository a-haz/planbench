#include "planbench/benchmark_report.hpp"

#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace planbench::benchmark {

void addTrial(Stats& stats, bool success, double length_m, double runtime_ms) {
  ++stats.trials;
  stats.runtime_sum += runtime_ms;
  if (success) {
    ++stats.successes;
    stats.length_sum += length_m;
  }
}

std::string formatDouble(double value, int digits) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(digits) << value;
  return out.str();
}

bool validAStar(const OccupancyGrid& grid, Vec2 start, Vec2 goal, const PlanResult& result) {
  if (!result.success) {
    return result.path.empty();
  }
  if (result.path.empty()) {
    return false;
  }
  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  if (!grid.worldToCell(start.x, start.y, sx, sy) || !grid.worldToCell(goal.x, goal.y, gx, gy)) {
    return false;
  }
  if (distance(result.path.front(), grid.cellCenter(sx, sy)) > 1e-6) {
    return false;
  }
  if (distance(result.path.back(), grid.cellCenter(gx, gy)) > 1e-6) {
    return false;
  }
  if (std::abs(result.length - polylineLength(result.path)) > 1e-6) {
    return false;
  }
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
    if (!grid.worldToCell(result.path[i - 1].x, result.path[i - 1].y, x0, y0) ||
        !grid.worldToCell(result.path[i].x, result.path[i].y, x1, y1)) {
      return false;
    }
    const int dx = x1 - x0;
    const int dy = y1 - y0;
    if (std::abs(dx) > 1 || std::abs(dy) > 1 || (dx == 0 && dy == 0)) {
      return false;
    }
    if (grid.occupied(x1, y1)) {
      return false;
    }
    if (dx != 0 && dy != 0 && (grid.occupied(x0 + dx, y0) || grid.occupied(x0, y0 + dy))) {
      return false;
    }
    if (!grid.segmentFree(result.path[i - 1], result.path[i])) {
      return false;
    }
  }
  return true;
}

bool validRRT(const OccupancyGrid& grid, Vec2 start, Vec2 goal, const RRTStarResult& result) {
  if (!result.success) {
    return result.path.empty();
  }
  if (result.path.size() < 2) {
    return false;
  }
  if (distance(result.path.front(), start) > 1e-6 || distance(result.path.back(), goal) > 1e-6) {
    return false;
  }
  if (std::abs(result.length - polylineLength(result.path)) > 1e-5) {
    return false;
  }
  if (result.length + 1e-6 < distance(start, goal)) {
    return false;
  }
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    if (!grid.segmentFree(result.path[i - 1], result.path[i])) {
      return false;
    }
  }
  return true;
}

bool validLattice(const OccupancyGrid& grid, Vec2 start, Vec2 goal,
                  const LatticeResult& result, const LatticeParams& params) {
  if (!result.success) {
    return result.path.empty();
  }
  if (result.path.size() != result.heading.size() || result.path.empty()) {
    return false;
  }
  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  if (!grid.worldToCell(start.x, start.y, sx, sy) || !grid.worldToCell(goal.x, goal.y, gx, gy)) {
    return false;
  }
  if (distance(result.path.front(), grid.cellCenter(sx, sy)) > 1e-6 ||
      distance(result.path.back(), grid.cellCenter(gx, gy)) > 1e-6) {
    return false;
  }
  if (result.length + 1e-6 < distance(start, goal)) {
    return false;
  }
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    if (!grid.rectangleFree(result.path[i].x, result.path[i].y, result.heading[i], params.body_length_m,
                            params.body_width_m)) {
      return false;
    }
  }
  return true;
}

std::string csvRow(const std::string& table, const std::string& planner, double density, int iterations, const Stats& stats) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << table << "," << planner << "," << formatDouble(density, 2) << "," << iterations << "," << stats.trials << ","
      << stats.successes << "," << formatDouble(static_cast<double>(stats.successes) / static_cast<double>(stats.trials), 6)
      << ",";
  if (stats.successes > 0) {
    out << formatDouble(stats.length_sum / static_cast<double>(stats.successes), 6);
  }
  out << "," << formatDouble(stats.runtime_sum / static_cast<double>(stats.trials), 6);
  return out.str();
}

}  // namespace planbench::benchmark
