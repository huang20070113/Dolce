#include "stl_to_grid_map/stl_mesh.hpp"

#include <array>
#include <algorithm>
#include <type_traits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace stl_to_grid_map {

namespace {

template <typename T>
T readLittleEndian(const unsigned char *bytes) {
  static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
  T value{};
  std::memcpy(&value, bytes, sizeof(T));
  return value;
}

StlMesh loadBinaryStl(const std::vector<unsigned char> &data, std::uint32_t count) {
  std::vector<Triangle> triangles;
  triangles.reserve(count);
  std::size_t offset = 84;
  for (std::uint32_t index = 0; index < count; ++index) {
    offset += 12;  // Stored normal; a normalized normal is recomputed when needed.
    Triangle triangle;
    Vec3 *vertices[] = {&triangle.a, &triangle.b, &triangle.c};
    for (Vec3 *vertex : vertices) {
      vertex->x = readLittleEndian<float>(&data[offset]);
      vertex->y = readLittleEndian<float>(&data[offset + 4]);
      vertex->z = readLittleEndian<float>(&data[offset + 8]);
      offset += 12;
    }
    offset += 2;  // Attribute byte count.
    triangles.push_back(triangle);
  }
  return StlMesh(std::move(triangles));
}

StlMesh loadAsciiStl(const std::string &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open STL file: " + path);
  }

  std::vector<Vec3> vertices;
  std::string token;
  while (input >> token) {
    if (token != "vertex") {
      continue;
    }
    Vec3 vertex;
    if (!(input >> vertex.x >> vertex.y >> vertex.z)) {
      throw std::runtime_error("invalid ASCII STL vertex in: " + path);
    }
    vertices.push_back(vertex);
  }

  if (vertices.empty() || vertices.size() % 3 != 0) {
    throw std::runtime_error("ASCII STL does not contain complete triangles: " + path);
  }

  std::vector<Triangle> triangles;
  triangles.reserve(vertices.size() / 3);
  for (std::size_t index = 0; index < vertices.size(); index += 3) {
    triangles.push_back(Triangle{vertices[index], vertices[index + 1],
                                 vertices[index + 2]});
  }
  return StlMesh(std::move(triangles));
}

}  // namespace

StlMesh::StlMesh(std::vector<Triangle> triangles)
    : triangles_(std::move(triangles)) {
  if (triangles_.empty()) {
    throw std::invalid_argument("STL mesh must contain at least one triangle");
  }
  for (const auto &t : triangles_) for (const auto &p : {t.a, t.b, t.c})
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::invalid_argument("STL contains non-finite coordinates");
}

StlMesh StlMesh::load(const std::string &path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) {
    throw std::runtime_error("cannot open STL file: " + path);
  }
  const auto end = input.tellg();
  if (end < 0) {
    throw std::runtime_error("cannot determine STL file size: " + path);
  }
  const auto file_size = static_cast<std::size_t>(end);
  input.seekg(0);

  std::vector<unsigned char> data(file_size);
  if (file_size > 0 &&
      !input.read(reinterpret_cast<char *>(data.data()),
                  static_cast<std::streamsize>(file_size))) {
    throw std::runtime_error("cannot read STL file: " + path);
  }

  if (file_size >= 84) {
    const std::uint32_t triangle_count = readLittleEndian<std::uint32_t>(&data[80]);
    const unsigned long long expected =
        84ULL + static_cast<unsigned long long>(triangle_count) * 50ULL;
    if (expected == file_size && triangle_count > 0) {
      return loadBinaryStl(data, triangle_count);
    }
  }
  return loadAsciiStl(path);
}

Bounds3 StlMesh::bounds() const {
  const double infinity = std::numeric_limits<double>::infinity();
  Bounds3 result{{infinity, infinity, infinity},
                 {-infinity, -infinity, -infinity}};
  for (const Triangle &triangle : triangles_) {
    const Vec3 vertices[] = {triangle.a, triangle.b, triangle.c};
    for (const Vec3 &vertex : vertices) {
      result.minimum.x = std::min(result.minimum.x, vertex.x);
      result.minimum.y = std::min(result.minimum.y, vertex.y);
      result.minimum.z = std::min(result.minimum.z, vertex.z);
      result.maximum.x = std::max(result.maximum.x, vertex.x);
      result.maximum.y = std::max(result.maximum.y, vertex.y);
      result.maximum.z = std::max(result.maximum.z, vertex.z);
    }
  }
  return result;
}

Vec3 triangleNormal(const Triangle &triangle) {
  const Vec3 first{triangle.b.x - triangle.a.x, triangle.b.y - triangle.a.y,
                   triangle.b.z - triangle.a.z};
  const Vec3 second{triangle.c.x - triangle.a.x, triangle.c.y - triangle.a.y,
                    triangle.c.z - triangle.a.z};
  Vec3 normal{first.y * second.z - first.z * second.y,
              first.z * second.x - first.x * second.z,
              first.x * second.y - first.y * second.x};
  const double length =
      std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
  if (length > 0.0) {
    normal.x /= length;
    normal.y /= length;
    normal.z /= length;
  }
  return normal;
}

}  // namespace stl_to_grid_map
