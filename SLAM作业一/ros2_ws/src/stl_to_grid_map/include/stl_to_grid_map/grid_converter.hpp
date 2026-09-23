#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "stl_to_grid_map/stl_mesh.hpp"

namespace stl_to_grid_map {

struct ConversionOptions {
  double resolution = 0.05;
  double robot_radius = 0.25;
  double slice_min_z = 0.10;
  double slice_max_z = 0.60;
  double scale = 0.001;
  double padding = 0.50;
  double max_abs_normal_z = 1.0;
  bool require_ground = false;
  double ground_tolerance = 0.03;
  bool terrain_mode = false;
  double max_support_z = 0.65;
  double max_slope_degrees = 25.0;
  double max_step_height = 0.06;
  double robot_height = 0.60;
};

struct OccupancyMap {
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<std::uint8_t> occupied;

  bool inBounds(int x, int y) const noexcept;
  bool isOccupied(int x, int y) const;
  std::size_t occupiedCount() const noexcept;
};

struct ConversionResult {
  OccupancyMap map;
  std::size_t total_triangles = 0;
  std::size_t selected_triangles = 0;
  std::size_t raw_occupied_cells = 0;
  std::size_t inflated_occupied_cells = 0;
  Bounds3 scaled_mesh_bounds;
};

ConversionResult convertMesh(const StlMesh &mesh, const ConversionOptions &options);
void writePgmAndYaml(const OccupancyMap &map, const std::string &output_prefix);

}  // namespace stl_to_grid_map
