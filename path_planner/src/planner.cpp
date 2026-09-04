#include "path_planner/planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>

namespace path_planner {
namespace {

constexpr int kStraightCost = 10;
constexpr int kDiagonalCost = 14;
constexpr int kDirections[][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
};

struct OpenNode {
  int f;
  int g;
  int index;
  bool operator>(const OpenNode& other) const {
    if (f != other.f) return f > other.f;
    if (g != other.g) return g < other.g;
    return index > other.index;
  }
};

int heuristic(Point from, Point to) {
  const int dx = std::abs(from.x - to.x);
  const int dy = std::abs(from.y - to.y);
  const int diagonal = std::min(dx, dy);
  return diagonal * kDiagonalCost + (std::max(dx, dy) - diagonal) * kStraightCost;
}

}  // namespace

Grid::Grid(int width, int height, std::vector<std::uint8_t> cells)
    : width_(width), height_(height), cells_(std::move(cells)) {
  if (width_ <= 0 || height_ <= 0) throw std::invalid_argument("map dimensions must be positive");
  const auto expected = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  if (cells_.size() != expected) throw std::invalid_argument("map cell count does not match dimensions");
  if (std::any_of(cells_.begin(), cells_.end(), [](std::uint8_t value) { return value > 1; })) {
    throw std::invalid_argument("map cells must be 0 or 1");
  }
}

bool Grid::in_bounds(Point point) const {
  return point.x >= 0 && point.y >= 0 && point.x < width_ && point.y < height_;
}

bool Grid::blocked(Point point) const {
  if (!in_bounds(point)) return true;
  const auto index = static_cast<std::size_t>(point.y) * static_cast<std::size_t>(width_) +
                     static_cast<std::size_t>(point.x);
  return cells_[index] != 0;
}

Grid load_map(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open map: " + path);
  long long width = 0;
  long long height = 0;
  if (!(input >> width >> height) || width <= 0 || height <= 0 ||
      width > std::numeric_limits<int>::max() || height > std::numeric_limits<int>::max() ||
      width > std::numeric_limits<int>::max() / height) {
    throw std::runtime_error("invalid map dimensions");
  }
  const auto count = static_cast<std::size_t>(width * height);
  std::vector<std::uint8_t> cells;
  cells.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    int value = -1;
    if (!(input >> value) || (value != 0 && value != 1)) {
      throw std::runtime_error("map must contain exactly width * height values of 0 or 1");
    }
    cells.push_back(static_cast<std::uint8_t>(value));
  }
  std::string extra;
  if (input >> extra) throw std::runtime_error("map contains extra data");
  return Grid(static_cast<int>(width), static_cast<int>(height), std::move(cells));
}

Grid inflate_obstacles(const Grid& map, int radius) {
  if (radius < 0) throw std::invalid_argument("robot_radius must be non-negative");
  if (radius == 0) return map;
  std::vector<std::uint8_t> cells = map.cells();
  const long long radius_squared = 1LL * radius * radius;
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.blocked(Point{x, y})) continue;
      bool unsafe = false;
      for (int dy = -radius; dy <= radius && !unsafe; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (1LL * dx * dx + 1LL * dy * dy <= radius_squared &&
              map.blocked(Point{x + dx, y + dy})) {
            unsafe = true;
            break;
          }
        }
      }
      if (unsafe) cells[static_cast<std::size_t>(y * map.width() + x)] = 1;
    }
  }
  return Grid(map.width(), map.height(), std::move(cells));
}

bool legal_step(const Grid& map, Point from, Point to) {
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (map.blocked(from) || map.blocked(to) || std::abs(dx) > 1 ||
      std::abs(dy) > 1 || (dx == 0 && dy == 0)) return false;
  if (dx != 0 && dy != 0) {
    return !map.blocked(Point{from.x + dx, from.y}) &&
           !map.blocked(Point{from.x, from.y + dy});
  }
  return true;
}

bool line_of_sight(const Grid& map, Point from, Point to) {
  if (map.blocked(from) || map.blocked(to)) return false;
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  const int nx = std::abs(dx);
  const int ny = std::abs(dy);
  const int sign_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
  const int sign_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
  Point current = from;
  int ix = 0;
  int iy = 0;
  while (ix < nx || iy < ny) {
    const long long decision = 1LL * (1 + 2 * ix) * ny - 1LL * (1 + 2 * iy) * nx;
    Point next = current;
    if (decision == 0) {
      next.x += sign_x;
      next.y += sign_y;
      ++ix;
      ++iy;
    } else if (decision < 0) {
      next.x += sign_x;
      ++ix;
    } else {
      next.y += sign_y;
      ++iy;
    }
    if (!legal_step(map, current, next)) return false;
    current = next;
  }
  return current == to;
}

std::vector<Point> simplify_path(const Grid& map, const std::vector<Point>& path) {
  if (path.size() < 3) return path;
  std::vector<Point> simplified{path.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t farthest = anchor + 1;
    for (std::size_t candidate = anchor + 2; candidate < path.size(); ++candidate) {
      if (line_of_sight(map, path[anchor], path[candidate])) farthest = candidate;
    }
    simplified.push_back(path[farthest]);
    anchor = farthest;
  }
  return simplified;
}

PlanResult plan(const Grid& map, Point start, Point goal) {
  if (!map.in_bounds(start)) throw std::invalid_argument("start is out of bounds");
  if (!map.in_bounds(goal)) throw std::invalid_argument("goal is out of bounds");
  if (map.blocked(start)) throw std::invalid_argument("start is occupied or inside inflated area");
  if (map.blocked(goal)) throw std::invalid_argument("goal is occupied or inside inflated area");

  PlanResult result;
  const int total = map.width() * map.height();
  std::vector<int> g_score(static_cast<std::size_t>(total), std::numeric_limits<int>::max());
  std::vector<int> parent(static_cast<std::size_t>(total), -1);
  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open;
  const auto index_of = [&map](Point point) { return point.y * map.width() + point.x; };
  const int start_index = index_of(start);
  g_score[start_index] = 0;
  open.push(OpenNode{heuristic(start, goal), 0, start_index});

  while (!open.empty()) {
    const OpenNode current = open.top();
    open.pop();
    if (current.g != g_score[current.index]) continue;
    const Point point{current.index % map.width(), current.index / map.width()};
    if (point == goal) {
      result.found = true;
      result.cost = current.g;
      for (int at = current.index; at != -1; at = parent[at]) {
        result.raw_path.push_back(Point{at % map.width(), at / map.width()});
      }
      std::reverse(result.raw_path.begin(), result.raw_path.end());
      result.simplified_path = simplify_path(map, result.raw_path);
      return result;
    }
    for (const auto& direction : kDirections) {
      const int dx = direction[0];
      const int dy = direction[1];
      const Point next{point.x + dx, point.y + dy};
      if (!legal_step(map, point, next)) continue;
      const int next_index = index_of(next);
      const int candidate = current.g + (dx == 0 || dy == 0 ? kStraightCost : kDiagonalCost);
      if (candidate < g_score[next_index]) {
        g_score[next_index] = candidate;
        parent[next_index] = current.index;
        open.push(OpenNode{candidate + heuristic(next, goal), candidate, next_index});
      }
    }
  }
  return result;
}

void print_result(std::ostream& output, const Grid& map, Point start,
                  Point goal, const PlanResult& result) {
  if (!result.found) {
    output << "NO_PATH\n";
    return;
  }
  output << "PATH_FOUND\nCOST " << result.cost << "\nLENGTH "
         << result.simplified_path.size() << '\n';
  for (const Point point : result.simplified_path) output << point.x << ' ' << point.y << '\n';
  std::vector<std::string> drawing(static_cast<std::size_t>(map.height()),
                                   std::string(static_cast<std::size_t>(map.width()), '.'));
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.blocked(Point{x, y})) drawing[y][x] = '#';
    }
  }
  for (const Point point : result.raw_path) drawing[point.y][point.x] = '*';
  drawing[start.y][start.x] = 'S';
  drawing[goal.y][goal.x] = 'G';
  output << "MAP\n";
  for (const auto& row : drawing) output << row << '\n';
}

}  // namespace path_planner
