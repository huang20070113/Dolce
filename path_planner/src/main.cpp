#include "path_planner/planner.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int parse_integer(const char* text, const std::string& name) {
  const std::string value(text);
  std::size_t used = 0;
  int result = 0;
  try {
    result = std::stoi(value, &used);
  } catch (const std::exception&) {
    throw std::invalid_argument(name + " must be an integer");
  }
  if (used != value.size()) throw std::invalid_argument(name + " must be an integer");
  return result;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    if (argc != 7) {
      throw std::invalid_argument(
          "usage: path_planner <map.txt> <start_x> <start_y> <goal_x> <goal_y> <robot_radius>");
    }
    const path_planner::Point start{parse_integer(argv[2], "start_x"),
                                    parse_integer(argv[3], "start_y")};
    const path_planner::Point goal{parse_integer(argv[4], "goal_x"),
                                   parse_integer(argv[5], "goal_y")};
    const int radius = parse_integer(argv[6], "robot_radius");
    const auto original = path_planner::load_map(argv[1]);
    const auto inflated = path_planner::inflate_obstacles(original, radius);
    const auto result = path_planner::plan(inflated, start, goal);
    path_planner::print_result(std::cout, inflated, start, goal, result);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "path_planner: " << error.what() << '\n';
    return 1;
  }
}
