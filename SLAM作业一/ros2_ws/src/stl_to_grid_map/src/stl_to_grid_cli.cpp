#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include "stl_to_grid_map/grid_converter.hpp"
#include "stl_to_grid_map/stl_mesh.hpp"

namespace {

double parseDouble(const char *text, const char *name) {
  const std::string value(text);
  std::size_t parsed = 0;
  double result = 0.0;
  try {
    result = std::stod(value, &parsed);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string(name) + " must be a number");
  }
  if (parsed != value.size()) {
    throw std::invalid_argument(std::string(name) + " must be a number");
  }
  return result;
}

}  // namespace

int main(int argc, char *argv[]) {
  if (argc != 9 && argc != 10) {
    std::cerr
        << "Usage: " << argv[0]
        << " input.stl output_prefix resolution robot_radius slice_min_z "
           "slice_max_z scale padding [mode=0:band,1:ground,2:terrain]\n";
    return 2;
  }

  try {
    stl_to_grid_map::ConversionOptions options;
    options.resolution = parseDouble(argv[3], "resolution");
    options.robot_radius = parseDouble(argv[4], "robot_radius");
    options.slice_min_z = parseDouble(argv[5], "slice_min_z");
    options.slice_max_z = parseDouble(argv[6], "slice_max_z");
    options.scale = parseDouble(argv[7], "scale");
    options.padding = parseDouble(argv[8], "padding");
    if (argc == 10) {
      const std::string flag(argv[9]);
      if (flag != "0" && flag != "1" && flag != "2") throw std::invalid_argument("mode must be 0, 1 or 2");
      options.require_ground = flag == "1";
      options.terrain_mode = flag == "2";
    }

    const auto mesh = stl_to_grid_map::StlMesh::load(argv[1]);
    const auto result = stl_to_grid_map::convertMesh(mesh, options);
    stl_to_grid_map::writePgmAndYaml(result.map, argv[2]);

    std::cout << "MAP_GENERATED\n";
    std::cout << "TRIANGLES_TOTAL " << result.total_triangles << '\n';
    std::cout << "TRIANGLES_SELECTED " << result.selected_triangles << '\n';
    std::cout << "MESH_BOUNDS_M " << result.scaled_mesh_bounds.minimum.x << ' '
              << result.scaled_mesh_bounds.minimum.y << ' '
              << result.scaled_mesh_bounds.minimum.z << ' '
              << result.scaled_mesh_bounds.maximum.x << ' '
              << result.scaled_mesh_bounds.maximum.y << ' '
              << result.scaled_mesh_bounds.maximum.z << '\n';
    std::cout << "MAP_SIZE " << result.map.width << ' ' << result.map.height
              << '\n';
    std::cout << "RAW_OCCUPIED " << result.raw_occupied_cells << '\n';
    std::cout << "INFLATED_OCCUPIED " << result.inflated_occupied_cells << '\n';
    std::cout << "OUTPUT_PREFIX " << argv[2] << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 2;
  }
}
