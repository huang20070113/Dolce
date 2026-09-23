#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "stl_to_grid_map/grid_converter.hpp"
#include "stl_to_grid_map/stl_mesh.hpp"

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void writeTestStl(const std::filesystem::path &path) {
  std::ofstream output(path);
  output << "solid test\n";
  output << "facet normal 0 0 1 outer loop\n";
  output << "vertex 0 0 0 vertex 2 0 0 vertex 2 2 0\n";
  output << "endloop endfacet\n";
  output << "facet normal 0 0 1 outer loop\n";
  output << "vertex 0 0 0 vertex 2 2 0 vertex 0 2 0\n";
  output << "endloop endfacet\n";
  output << "facet normal 1 0 0 outer loop\n";
  output << "vertex 1 0 0 vertex 1 2 0 vertex 1 2 1\n";
  output << "endloop endfacet\n";
  output << "facet normal 1 0 0 outer loop\n";
  output << "vertex 1 0 0 vertex 1 2 1 vertex 1 0 1\n";
  output << "endloop endfacet\n";
  output << "endsolid test\n";
}

void runTest() {
  const auto temporary = std::filesystem::temp_directory_path();
  const auto stl_path = temporary / "stl_grid_core_test.stl";
  const auto output_prefix = temporary / "stl_grid_core_test_map";
  writeTestStl(stl_path);

  const auto mesh = stl_to_grid_map::StlMesh::load(stl_path.string());
  require(mesh.triangles().size() == 4, "ASCII STL parser should load four triangles");

  stl_to_grid_map::ConversionOptions options;
  options.resolution = 0.25;
  options.robot_radius = 0.25;
  options.slice_min_z = 0.10;
  options.slice_max_z = 0.80;
  options.scale = 1.0;
  options.padding = 0.25;
  options.max_abs_normal_z = 0.85;

  const auto result = stl_to_grid_map::convertMesh(mesh, options);
  require(result.selected_triangles == 2,
          "horizontal floor triangles should be filtered out");
  require(result.raw_occupied_cells > 0, "vertical wall should create occupied cells");
  require(result.inflated_occupied_cells > result.raw_occupied_cells,
          "robot radius should inflate the wall");

  stl_to_grid_map::writePgmAndYaml(result.map, output_prefix.string());
  require(std::filesystem::file_size(output_prefix.string() + ".pgm") > 10,
          "PGM output should not be empty");
  require(std::filesystem::file_size(output_prefix.string() + ".yaml") > 10,
          "YAML output should not be empty");

  std::filesystem::remove(stl_path);
  std::filesystem::remove(output_prefix.string() + ".pgm");
  std::filesystem::remove(output_prefix.string() + ".yaml");
}

void regressionTest() {
  const auto path = std::filesystem::temp_directory_path() / "stl_grid_regression.stl";
  std::ofstream out(path);
  out << "solid regression\n";
  // A floor, a raised horizontal platform and an inclined face reaching x=4 above the robot.
  for (const std::string vertices : {
       "0 0 0 vertex 5 0 0 vertex 5 5 0",
       "0 0 0 vertex 5 5 0 vertex 0 5 0",
       "1 1 0.3 vertex 2 1 0.3 vertex 1 2 0.3",
       "3 0 0 vertex 4 0 2 vertex 4 1 2"})
    out << "facet normal 0 0 1 outer loop vertex " << vertices << " endloop endfacet\n";
  out << "endsolid regression\n";
  out.close();
  stl_to_grid_map::ConversionOptions options;
  options.scale = 1;
  options.resolution = 0.05;
  options.padding = 0.5;
  options.robot_radius = 0;
  options.slice_min_z = 0.1;
  options.slice_max_z = 0.5;
  options.require_ground = true;
  auto mesh = stl_to_grid_map::StlMesh::load(path.string());
  auto result = stl_to_grid_map::convertMesh(mesh, options);
  auto occupied = [&](double x, double y) {
    return result.map.isOccupied(static_cast<int>((x-result.map.origin_x)/options.resolution),
                                 static_cast<int>((y-result.map.origin_y)/options.resolution));
  };
  require(occupied(1.2, 1.2), "raised platform must be blocked");
  require(!occupied(3.9, 0.3), "geometry above collision band must be clipped away");
  require(occupied(-0.3, 1), "unsupported exterior must be blocked");
  require(!occupied(2.5, 2.5), "supported flat ground must remain free");
  options.resolution = std::numeric_limits<double>::infinity();
  bool rejected = false;
  try { stl_to_grid_map::convertMesh(mesh, options); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "non-finite resolution must be rejected");
  std::filesystem::remove(path);
}

void terrainTest() {
  using namespace stl_to_grid_map;
  std::vector<Triangle> triangles;
  auto quad=[&](Vec3 a,Vec3 b,Vec3 c,Vec3 d) {
    triangles.push_back({a,b,c});triangles.push_back({a,c,d});
  };
  quad({0,0,0},{2,0,0},{2,4,0},{0,4,0});
  quad({2,0,0},{4,0,0.4},{4,4,0.4},{2,4,0}); // 11.3 degree ramp
  quad({4,0,0.4},{6,0,0.4},{6,4,0.4},{4,4,0.4});
  // Obstacle on raised platform, beyond the allowed support height.
  quad({4.8,1,1.0},{5.5,1,1.0},{5.5,2,1.0},{4.8,2,1.0});
  ConversionOptions options;
  options.scale=1;options.resolution=0.05;options.robot_radius=0.1;
  options.terrain_mode=true;
  auto result=convertMesh(StlMesh(triangles),options);
  auto blocked=[&](double x,double y) {
    return result.map.isOccupied(static_cast<int>((x-result.map.origin_x)/options.resolution),
                                  static_cast<int>((y-result.map.origin_y)/options.resolution));
  };
  require(!blocked(1,3), "flat ground must be free");
  require(!blocked(3,3), "gentle ramp must be free");
  require(!blocked(5,3), "platform reached through ramp must be free");
  require(blocked(5.1,1.5), "overhead obstacle within robot height must be blocked");
  options.max_slope_degrees=5;
  auto steep=convertMesh(StlMesh(triangles),options);
  require(steep.map.isOccupied(static_cast<int>((3-steep.map.origin_x)/options.resolution),
                              static_cast<int>((3-steep.map.origin_y)/options.resolution)),
          "ramp exceeding robot slope limit must be blocked");
  // Replace ramp with a 0.4 m step; elevated island must not connect to floor.
  triangles.clear();
  quad({0,0,0},{2,0,0},{2,4,0},{0,4,0});
  quad({2,0,0.4},{4,0,0.4},{4,4,0.4},{2,4,0.4});
  options.max_slope_degrees=25;
  auto step=convertMesh(StlMesh(triangles),options);
  require(step.map.isOccupied(static_cast<int>((2-step.map.origin_x)/options.resolution),
                             static_cast<int>((2-step.map.origin_y)/options.resolution)),
          "large step must be blocked");
}

}  // namespace

int main() {
  try {
    runTest();
    regressionTest();
    terrainTest();
    std::cout << "[PASS] STL parsing, filtering, inflation, and map output\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "[FAIL] " << error.what() << '\n';
    return 1;
  }
}

