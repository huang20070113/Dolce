#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace stl_to_grid_map {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Triangle {
  Vec3 a;
  Vec3 b;
  Vec3 c;
};

struct Bounds3 {
  Vec3 minimum;
  Vec3 maximum;
};

class StlMesh {
public:
  explicit StlMesh(std::vector<Triangle> triangles);

  static StlMesh load(const std::string &path);

  const std::vector<Triangle> &triangles() const noexcept { return triangles_; }
  Bounds3 bounds() const;

private:
  std::vector<Triangle> triangles_;
};

Vec3 triangleNormal(const Triangle &triangle);

}  // namespace stl_to_grid_map
