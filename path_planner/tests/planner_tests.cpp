#include "path_planner/planner.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

using path_planner::Grid;
using path_planner::Point;

int main() {
  {
    Grid grid(5, 5, std::vector<std::uint8_t>(25, 0));
    const auto result = path_planner::plan(grid, {0, 0}, {4, 4});
    assert(result.found && result.cost == 56 && result.raw_path.size() == 5);
    assert((result.simplified_path.front() == Point{0, 0}));
    assert((result.simplified_path.back() == Point{4, 4}));
  }
  {
    Grid grid(2, 2, {0, 1, 1, 0});
    assert(!path_planner::plan(grid, Point{0, 0}, Point{1, 1}).found);
    assert(!path_planner::line_of_sight(grid, Point{0, 0}, Point{1, 1}));
  }
  {
    Grid grid(5, 5, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    const auto inflated = path_planner::inflate_obstacles(grid, 1);
    assert(inflated.blocked(Point{2, 2}) && inflated.blocked(Point{1, 2}));
    assert(inflated.blocked(Point{2, 1}) && !inflated.blocked(Point{1, 1}));
  }
  {
    Grid grid(3, 3, std::vector<std::uint8_t>(9, 0));
    const auto result = path_planner::plan(grid, Point{1, 1}, Point{1, 1});
    assert(result.found && result.cost == 0 && result.raw_path.size() == 1);
  }
  {
    Grid grid(2, 2, std::vector<std::uint8_t>(4, 0));
    bool thrown = false;
    try {
      (void)path_planner::plan(grid, {-1, 0}, {1, 1});
    } catch (const std::invalid_argument&) {
      thrown = true;
    }
    assert(thrown);
  }
  std::cout << "All path planner tests passed.\n";
}
