#include "planbench/planbench.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Config {
  int width = 32;
  int height = 32;
  double resolution = 0.50;
  std::vector<double> densities{0.10, 0.20, 0.30};
  int seed_count = 20;
  std::uint32_t seed0 = 1;
  int clearance_cells = 3;
  double rrt_step = 0.75;
  double rrt_goal_radius = 0.75;
  double rrt_goal_bias = 0.10;
  std::vector<int> rrt_checkpoints{250, 500, 1000, 2000};
  int rrt_summary_iterations = 1000;
  double turning_radius = 1.0;
  double body_length = 0.45;
  double body_width = 0.28;
  std::string output = "docs/benchmark.csv";
  std::string paths = "docs/example_paths.csv";
};

Config smallConfig() {
  Config config;
  config.width = 20;
  config.height = 20;
  config.resolution = 0.5;
  config.densities = {0.10};
  config.seed_count = 2;
  config.clearance_cells = 2;
  config.rrt_step = 0.75;
  config.rrt_goal_radius = 0.75;
  config.rrt_checkpoints = {40, 80};
  config.rrt_summary_iterations = 80;
  config.turning_radius = 1.0;
  config.body_length = 0.60;
  config.body_width = 0.35;
  return config;
}

struct Stats {
  int trials = 0;
  int successes = 0;
  double length_sum = 0.0;
  double runtime_sum = 0.0;
};

struct ExampleChoice {
  bool valid = false;
  int successes = -1;
  double astar_length = -1.0;
  double density = 0.0;
  std::uint32_t seed = 0;
};

void printUsage() {
  std::cerr << "usage: planbench_benchmark [--small] [--output PATH] [--paths PATH]\n";
}

std::string formatDouble(double value, int digits) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(digits) << value;
  return out.str();
}

bool writeParent(const std::filesystem::path& path) {
  const auto parent = path.parent_path();
  if (parent.empty()) {
    return true;
  }
  std::error_code error;
  std::filesystem::create_directories(parent, error);
  return !error;
}

bool validAStar(const planbench::OccupancyGrid& grid, planbench::Vec2 start, planbench::Vec2 goal,
                const planbench::PlanResult& result) {
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
  if (planbench::distance(result.path.front(), grid.cellCenter(sx, sy)) > 1e-6) {
    return false;
  }
  if (planbench::distance(result.path.back(), grid.cellCenter(gx, gy)) > 1e-6) {
    return false;
  }
  if (std::abs(result.length - planbench::polylineLength(result.path)) > 1e-6) {
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

bool validRRT(const planbench::OccupancyGrid& grid, planbench::Vec2 start, planbench::Vec2 goal,
              const planbench::RRTStarResult& result) {
  if (!result.success) {
    return result.path.empty();
  }
  if (result.path.size() < 2) {
    return false;
  }
  if (planbench::distance(result.path.front(), start) > 1e-6 || planbench::distance(result.path.back(), goal) > 1e-6) {
    return false;
  }
  if (std::abs(result.length - planbench::polylineLength(result.path)) > 1e-5) {
    return false;
  }
  if (result.length + 1e-6 < planbench::distance(start, goal)) {
    return false;
  }
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    if (!grid.segmentFree(result.path[i - 1], result.path[i])) {
      return false;
    }
  }
  return true;
}

bool validLattice(const planbench::OccupancyGrid& grid, planbench::Vec2 start, planbench::Vec2 goal,
                  const planbench::LatticeResult& result, const planbench::LatticeParams& params) {
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
  if (planbench::distance(result.path.front(), grid.cellCenter(sx, sy)) > 1e-6 ||
      planbench::distance(result.path.back(), grid.cellCenter(gx, gy)) > 1e-6) {
    return false;
  }
  if (result.length + 1e-6 < planbench::distance(start, goal)) {
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

const planbench::RRTStarSnapshot* snapshotAt(const planbench::RRTStarResult& result, int iterations) {
  for (const auto& snap : result.snapshots) {
    if (snap.iterations == iterations) {
      return &snap;
    }
  }
  return nullptr;
}

void considerExample(ExampleChoice& best, double density, std::uint32_t seed, bool astar_ok, bool rrt_ok,
                     bool lattice_ok, double astar_length) {
  const int successes = static_cast<int>(astar_ok) + static_cast<int>(rrt_ok) + static_cast<int>(lattice_ok);
  if (successes <= 0) {
    return;
  }
  const double length = astar_ok ? astar_length : 0.0;
  const bool better = !best.valid || successes > best.successes ||
                      (successes == best.successes && length > best.astar_length + 1e-9);
  if (better) {
    best.valid = true;
    best.successes = successes;
    best.astar_length = length;
    best.density = density;
    best.seed = seed;
  }
}

bool writeExample(const Config& config, const ExampleChoice& choice) {
  if (!writeParent(config.paths)) {
    std::cerr << "could not create " << config.paths << "\n";
    return false;
  }
  const double density = choice.valid ? choice.density : config.densities.front();
  const std::uint32_t seed = choice.valid ? choice.seed : config.seed0;
  const auto map =
      planbench::generateMap(config.width, config.height, config.resolution, density, seed, config.clearance_cells);

  planbench::RRTStarParams rrt_params;
  rrt_params.max_iterations = config.rrt_summary_iterations;
  rrt_params.step_size = config.rrt_step;
  rrt_params.goal_radius = config.rrt_goal_radius;
  rrt_params.goal_sample_probability = config.rrt_goal_bias;
  rrt_params.seed = seed + 7919u;

  planbench::LatticeParams lattice_params;
  lattice_params.turning_radius_m = config.turning_radius;
  lattice_params.body_length_m = config.body_length;
  lattice_params.body_width_m = config.body_width;

  const auto astar = planbench::AStarPlanner{}.plan(map.grid, map.start, map.goal);
  const auto rrt = planbench::RRTStarPlanner{}.plan(map.grid, map.start, map.goal, rrt_params);
  const auto lattice = planbench::StateLatticePlanner{}.plan(map.grid, map.start, map.goal, lattice_params);
  if (!validAStar(map.grid, map.start, map.goal, astar) || !validRRT(map.grid, map.start, map.goal, rrt) ||
      !validLattice(map.grid, map.start, map.goal, lattice, lattice_params)) {
    std::cerr << "example path failed validation\n";
    return false;
  }

  std::ofstream out(config.paths);
  if (!out) {
    std::cerr << "could not write " << config.paths << "\n";
    return false;
  }
  out.imbue(std::locale::classic());
  out << "kind,series,a,b,c\n";
  out << "meta,width," << config.width << ",0,0\n";
  out << "meta,height," << config.height << ",0,0\n";
  out << "meta,resolution," << formatDouble(config.resolution, 6) << ",0,0\n";
  out << "meta,density," << formatDouble(density, 2) << ",0,0\n";
  out << "meta,seed," << seed << ",0,0\n";
  out << "meta,start_x," << formatDouble(map.start.x, 6) << ",0,0\n";
  out << "meta,start_y," << formatDouble(map.start.y, 6) << ",0,0\n";
  out << "meta,goal_x," << formatDouble(map.goal.x, 6) << ",0,0\n";
  out << "meta,goal_y," << formatDouble(map.goal.y, 6) << ",0,0\n";
  for (int y = 0; y < map.grid.height(); ++y) {
    for (int x = 0; x < map.grid.width(); ++x) {
      if (map.grid.occupied(x, y)) {
        out << "cell,occupied," << x << "," << y << ",0\n";
      }
    }
  }
  const auto write_path = [&](const std::string& name, const std::vector<planbench::Vec2>& path,
                              const std::vector<double>* heading) {
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double theta = heading != nullptr ? (*heading)[i] : 0.0;
      out << "path," << name << "," << formatDouble(path[i].x, 6) << "," << formatDouble(path[i].y, 6) << ","
          << formatDouble(theta, 6) << "\n";
    }
  };
  if (astar.success) {
    write_path("astar", astar.path, nullptr);
  }
  if (rrt.success) {
    write_path("rrt_star", rrt.path, nullptr);
  }
  if (lattice.success) {
    write_path("state_lattice", lattice.path, &lattice.heading);
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

}  // namespace

int main(int argc, char** argv) {
  Config config;
  bool small = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--small") {
      small = true;
    } else if (arg == "--output" && i + 1 < argc) {
      config.output = argv[++i];
    } else if (arg == "--paths" && i + 1 < argc) {
      config.paths = argv[++i];
    } else if (arg == "--help") {
      printUsage();
      return 0;
    } else {
      printUsage();
      return 2;
    }
  }
  if (small) {
    const std::string output = config.output;
    const std::string paths = config.paths;
    config = smallConfig();
    config.output = output;
    config.paths = paths;
  }

  std::vector<std::pair<double, Stats>> astar_rows;
  std::vector<std::pair<double, Stats>> lattice_rows;
  std::vector<std::pair<double, Stats>> rrt_summary_rows;
  std::vector<std::vector<Stats>> rrt_curve(config.densities.size(), std::vector<Stats>(config.rrt_checkpoints.size()));
  ExampleChoice example;

  for (std::size_t d = 0; d < config.densities.size(); ++d) {
    const double density = config.densities[d];
    Stats astar_stats;
    Stats lattice_stats;
    Stats rrt_summary;
    for (int s = 0; s < config.seed_count; ++s) {
      const std::uint32_t seed = config.seed0 + static_cast<std::uint32_t>(s);
      const auto map = planbench::generateMap(config.width, config.height, config.resolution, density, seed,
                                              config.clearance_cells);

      const auto astar_begin = std::chrono::steady_clock::now();
      const auto astar = planbench::AStarPlanner{}.plan(map.grid, map.start, map.goal);
      const double astar_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - astar_begin).count();
      if (!validAStar(map.grid, map.start, map.goal, astar)) {
        std::cerr << "A* path failed validation at density " << density << " seed " << seed << "\n";
        return 1;
      }

      planbench::RRTStarParams rrt_params;
      rrt_params.max_iterations = config.rrt_checkpoints.back();
      rrt_params.step_size = config.rrt_step;
      rrt_params.goal_radius = config.rrt_goal_radius;
      rrt_params.goal_sample_probability = config.rrt_goal_bias;
      rrt_params.seed = seed + 7919u;
      rrt_params.snapshot_iterations = config.rrt_checkpoints;
      const auto rrt = planbench::RRTStarPlanner{}.plan(map.grid, map.start, map.goal, rrt_params);
      if (!validRRT(map.grid, map.start, map.goal, rrt)) {
        std::cerr << "RRT* path failed validation at density " << density << " seed " << seed << "\n";
        return 1;
      }
      if (rrt.snapshots.size() != config.rrt_checkpoints.size()) {
        std::cerr << "RRT* snapshot count mismatch\n";
        return 1;
      }

      planbench::LatticeParams lattice_params;
      lattice_params.turning_radius_m = config.turning_radius;
      lattice_params.body_length_m = config.body_length;
      lattice_params.body_width_m = config.body_width;
      const auto lattice_begin = std::chrono::steady_clock::now();
      const auto lattice = planbench::StateLatticePlanner{}.plan(map.grid, map.start, map.goal, lattice_params);
      const double lattice_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lattice_begin).count();
      if (!validLattice(map.grid, map.start, map.goal, lattice, lattice_params)) {
        std::cerr << "state lattice path failed validation at density " << density << " seed " << seed << "\n";
        return 1;
      }

      ++astar_stats.trials;
      astar_stats.runtime_sum += astar_ms;
      if (astar.success) {
        ++astar_stats.successes;
        astar_stats.length_sum += astar.length;
      }

      ++lattice_stats.trials;
      lattice_stats.runtime_sum += lattice_ms;
      if (lattice.success) {
        ++lattice_stats.successes;
        lattice_stats.length_sum += lattice.length;
      }

      for (std::size_t c = 0; c < config.rrt_checkpoints.size(); ++c) {
        const auto* snap = snapshotAt(rrt, config.rrt_checkpoints[c]);
        if (snap == nullptr) {
          std::cerr << "missing RRT* snapshot\n";
          return 1;
        }
        auto& bucket = rrt_curve[d][c];
        ++bucket.trials;
        bucket.runtime_sum += snap->runtime_ms;
        if (snap->success) {
          ++bucket.successes;
          bucket.length_sum += snap->path_length;
        }
        if (config.rrt_checkpoints[c] == config.rrt_summary_iterations) {
          ++rrt_summary.trials;
          rrt_summary.runtime_sum += snap->runtime_ms;
          if (snap->success) {
            ++rrt_summary.successes;
            rrt_summary.length_sum += snap->path_length;
          }
        }
      }

      const auto* summary_snap = snapshotAt(rrt, config.rrt_summary_iterations);
      considerExample(example, density, seed, astar.success, summary_snap != nullptr && summary_snap->success,
                      lattice.success, astar.length);
      std::cout << "density " << formatDouble(density, 2) << " seed " << seed << " astar " << astar.success
                << " rrt " << (summary_snap != nullptr && summary_snap->success) << " lattice " << lattice.success
                << "\n";
    }
    astar_rows.push_back({density, astar_stats});
    lattice_rows.push_back({density, lattice_stats});
    rrt_summary_rows.push_back({density, rrt_summary});
  }

  if (!writeParent(config.output)) {
    std::cerr << "could not create " << config.output << "\n";
    return 1;
  }
  std::ofstream out(config.output);
  if (!out) {
    std::cerr << "could not write " << config.output << "\n";
    return 1;
  }
  out.imbue(std::locale::classic());
  out << "table,planner,density,iterations,trials,successes,success_rate,mean_path_length_m,mean_runtime_ms\n";
  std::vector<std::string> lines;
  for (std::size_t i = 0; i < config.densities.size(); ++i) {
    lines.push_back(csvRow("summary", "astar", astar_rows[i].first, 0, astar_rows[i].second));
    lines.push_back(csvRow("summary", "rrt_star", rrt_summary_rows[i].first, config.rrt_summary_iterations,
                           rrt_summary_rows[i].second));
    lines.push_back(csvRow("summary", "state_lattice", lattice_rows[i].first, 0, lattice_rows[i].second));
  }
  for (std::size_t d = 0; d < config.densities.size(); ++d) {
    for (std::size_t c = 0; c < config.rrt_checkpoints.size(); ++c) {
      lines.push_back(csvRow("rrt_budget", "rrt_star", config.densities[d], config.rrt_checkpoints[c], rrt_curve[d][c]));
    }
  }
  for (const auto& line : lines) {
    out << line << "\n";
    std::cout << line << "\n";
  }
  out.close();

  if (!writeExample(config, example)) {
    return 1;
  }
  return 0;
}
