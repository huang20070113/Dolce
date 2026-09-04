#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace path_planner {

struct Point {
  int x = 0;
  int y = 0;
  bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

class Grid {
 public:
  Grid(int width, int height, std::vector<std::uint8_t> cells);
  int width() const { return width_; }
  int height() const { return height_; }
  bool in_bounds(Point point) const;
  bool blocked(Point point) const;
  const std::vector<std::uint8_t>& cells() const { return cells_; }

 private:
  int width_;
  int height_;
  std::vector<std::uint8_t> cells_;
};

struct PlanResult {
  bool found = false;
  int cost = 0;
  std::vector<Point> raw_path;
  std::vector<Point> simplified_path;
};

Grid load_map(const std::string& path);
Grid inflate_obstacles(const Grid& map, int radius);
bool legal_step(const Grid& map, Point from, Point to);
bool line_of_sight(const Grid& map, Point from, Point to);
PlanResult plan(const Grid& map, Point start, Point goal);
std::vector<Point> simplify_path(const Grid& map, const std::vector<Point>& path);
void print_result(std::ostream& output, const Grid& map, Point start,
                  Point goal, const PlanResult& result);

}  // namespace path_planner
