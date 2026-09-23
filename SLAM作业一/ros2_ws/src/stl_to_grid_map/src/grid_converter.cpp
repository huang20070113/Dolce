#include "stl_to_grid_map/grid_converter.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

namespace stl_to_grid_map {

namespace {

constexpr std::size_t kMaximumCells = 100000000;

std::size_t cellIndex(const OccupancyMap &map, int x, int y) {
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(map.width) +
         static_cast<std::size_t>(x);
}

Vec3 scaled(const Vec3 &point, double scale) {
  return Vec3{point.x * scale, point.y * scale, point.z * scale};
}

Triangle scaled(const Triangle &triangle, double scale) {
  return Triangle{scaled(triangle.a, scale), scaled(triangle.b, scale),
                  scaled(triangle.c, scale)};
}

void mark(OccupancyMap &map, int x, int y) {
  if (map.inBounds(x, y)) {
    map.occupied[cellIndex(map, x, y)] = 1;
  }
}

std::pair<int, int> worldToCell(const OccupancyMap &map, double x, double y) {
  return {static_cast<int>(std::floor((x - map.origin_x) / map.resolution)),
          static_cast<int>(std::floor((y - map.origin_y) / map.resolution))};
}

void rasterizeLine(OccupancyMap &map, int x0, int y0, int x1, int y1) {
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  while (true) {
    mark(map, x0, y0);
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

double signedArea(double ax, double ay, double bx, double by, double px,
                  double py) {
  return (px - bx) * (ay - by) - (ax - bx) * (py - by);
}

bool pointInsideTriangle(double x, double y, const Triangle &triangle) {
  const double d1 = signedArea(triangle.a.x, triangle.a.y, triangle.b.x,
                               triangle.b.y, x, y);
  const double d2 = signedArea(triangle.b.x, triangle.b.y, triangle.c.x,
                               triangle.c.y, x, y);
  const double d3 = signedArea(triangle.c.x, triangle.c.y, triangle.a.x,
                               triangle.a.y, x, y);
  const bool has_negative = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
  const bool has_positive = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
  return !(has_negative && has_positive);
}

void rasterizeTriangle(OccupancyMap &map, const Triangle &triangle) {
  const auto a = worldToCell(map, triangle.a.x, triangle.a.y);
  const auto b = worldToCell(map, triangle.b.x, triangle.b.y);
  const auto c = worldToCell(map, triangle.c.x, triangle.c.y);
  rasterizeLine(map, a.first, a.second, b.first, b.second);
  rasterizeLine(map, b.first, b.second, c.first, c.second);
  rasterizeLine(map, c.first, c.second, a.first, a.second);

  const double projected_twice_area =
      std::abs((triangle.b.x - triangle.a.x) *
                   (triangle.c.y - triangle.a.y) -
               (triangle.b.y - triangle.a.y) *
                   (triangle.c.x - triangle.a.x));
  if (projected_twice_area < 1e-12) {
    return;
  }

  const int minimum_x = std::max(0, std::min({a.first, b.first, c.first}));
  const int maximum_x =
      std::min(map.width - 1, std::max({a.first, b.first, c.first}));
  const int minimum_y = std::max(0, std::min({a.second, b.second, c.second}));
  const int maximum_y =
      std::min(map.height - 1, std::max({a.second, b.second, c.second}));

  for (int y = minimum_y; y <= maximum_y; ++y) {
    for (int x = minimum_x; x <= maximum_x; ++x) {
      const double world_x = map.origin_x + (x + 0.5) * map.resolution;
      const double world_y = map.origin_y + (y + 0.5) * map.resolution;
      if (pointInsideTriangle(world_x, world_y, triangle)) {
        mark(map, x, y);
      }
    }
  }
}

// Visit projected cells with the triangle's local height interval. Nearly
// vertical faces retain their complete height interval for conservative collision.
template<class Visitor>
void visitSurface(const OccupancyMap &map, const Triangle &t, Visitor visitor) {
  auto a = worldToCell(map,t.a.x,t.a.y), b = worldToCell(map,t.b.x,t.b.y),
       c = worldToCell(map,t.c.x,t.c.y);
  const auto normal = triangleNormal(t);
  const double low = std::min({t.a.z,t.b.z,t.c.z});
  const double high = std::max({t.a.z,t.b.z,t.c.z});
  auto visit = [&](int x,int y) {
    if (!map.inBounds(x,y)) return;
    double lo=low, hi=high;
    if (std::abs(normal.z)>1e-6) {
      double wx=map.origin_x+(x+0.5)*map.resolution;
      double wy=map.origin_y+(y+0.5)*map.resolution;
      double z=t.a.z-(normal.x*(wx-t.a.x)+normal.y*(wy-t.a.y))/normal.z;
      lo=hi=std::clamp(z,low,high);
    }
    visitor(cellIndex(map,x,y),lo,hi);
  };
  auto line=[&](std::pair<int,int> p,std::pair<int,int> q) {
    int dx=std::abs(q.first-p.first), dy=-std::abs(q.second-p.second);
    int sx=p.first<q.first?1:-1, sy=p.second<q.second?1:-1, error=dx+dy;
    while(true) {
      visit(p.first,p.second);
      if(p==q) break;
      int e=2*error;
      if(e>=dy) {error+=dy;p.first+=sx;}
      if(e<=dx) {error+=dx;p.second+=sy;}
    }
  };
  line(a,b);line(b,c);line(c,a);
  if(std::abs(normal.z)<1e-6) return;
  for(int y=std::max(0,std::min({a.second,b.second,c.second}));
      y<=std::min(map.height-1,std::max({a.second,b.second,c.second}));++y)
    for(int x=std::max(0,std::min({a.first,b.first,c.first}));
        x<=std::min(map.width-1,std::max({a.first,b.first,c.first}));++x)
      if(pointInsideTriangle(map.origin_x+(x+0.5)*map.resolution,
                             map.origin_y+(y+0.5)*map.resolution,t)) visit(x,y);
}

void inflate(OccupancyMap &map, double robot_radius) {
  if (robot_radius <= 0.0) {
    return;
  }
  const int radius_cells =
      static_cast<int>(std::ceil(robot_radius / map.resolution));
  const double radius_squared = robot_radius * robot_radius;
  const auto original = map.occupied;

  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      if (original[cellIndex(map, x, y)] == 0) {
        continue;
      }
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const double center_distance_x = dx * map.resolution;
          const double center_distance_y = dy * map.resolution;
          if (center_distance_x * center_distance_x +
                  center_distance_y * center_distance_y <=
              radius_squared + 1e-12) {
            mark(map, x + dx, y + dy);
          }
        }
      }
    }
  }
}

// Sutherland-Hodgman clipping: project only geometry in the collision band.
std::vector<Vec3> clipHeight(const std::vector<Vec3> &input, double z, bool above) {
  std::vector<Vec3> output;
  if (input.empty()) return output;
  Vec3 previous = input.back();
  bool previous_inside = above ? previous.z >= z : previous.z <= z;
  for (const Vec3 &current : input) {
    const bool inside = above ? current.z >= z : current.z <= z;
    if (inside != previous_inside) {
      const double t = (z - previous.z) / (current.z - previous.z);
      output.push_back({previous.x + t * (current.x - previous.x),
                        previous.y + t * (current.y - previous.y), z});
    }
    if (inside) output.push_back(current);
    previous = current;
    previous_inside = inside;
  }
  return output;
}

void validateOptions(const ConversionOptions &options) {
  for (double value : {options.resolution, options.robot_radius, options.padding,
       options.scale, options.slice_min_z, options.slice_max_z,
       options.max_abs_normal_z, options.ground_tolerance, options.max_support_z,
       options.max_slope_degrees, options.max_step_height, options.robot_height}) {
    if (!std::isfinite(value)) throw std::invalid_argument("parameters must be finite");
  }
  if (options.ground_tolerance < 0) throw std::invalid_argument("invalid ground tolerance");
  if(options.max_support_z<0 || options.max_slope_degrees<0 || options.max_slope_degrees>=90 ||
     options.max_step_height<0 || options.robot_height<=options.max_step_height)
    throw std::invalid_argument("invalid terrain or robot dimensions");
  if (!(options.resolution > 0.0)) {
    throw std::invalid_argument("resolution must be positive");
  }
  if (options.robot_radius < 0.0 || options.padding < 0.0) {
    throw std::invalid_argument("robot_radius and padding must not be negative");
  }
  if (!(options.scale > 0.0)) {
    throw std::invalid_argument("scale must be positive");
  }
  if (options.slice_min_z > options.slice_max_z) {
    throw std::invalid_argument("slice_min_z must not exceed slice_max_z");
  }
  if (options.max_abs_normal_z < 0.0 || options.max_abs_normal_z > 1.0) {
    throw std::invalid_argument("max_abs_normal_z must be in [0, 1]");
  }
}

}  // namespace

bool OccupancyMap::inBounds(int x, int y) const noexcept {
  return x >= 0 && x < width && y >= 0 && y < height;
}

bool OccupancyMap::isOccupied(int x, int y) const {
  if (!inBounds(x, y)) {
    throw std::out_of_range("occupancy cell is outside the map");
  }
  return occupied[cellIndex(*this, x, y)] != 0;
}

std::size_t OccupancyMap::occupiedCount() const noexcept {
  return static_cast<std::size_t>(
      std::count(occupied.begin(), occupied.end(), std::uint8_t{1}));
}

ConversionResult convertMesh(const StlMesh &mesh,
                             const ConversionOptions &options) {
  validateOptions(options);
  const Bounds3 raw_bounds = mesh.bounds();
  const Bounds3 bounds{scaled(raw_bounds.minimum, options.scale),
                       scaled(raw_bounds.maximum, options.scale)};

  const double origin_x =
      std::floor((bounds.minimum.x - options.padding) / options.resolution) *
      options.resolution;
  const double origin_y =
      std::floor((bounds.minimum.y - options.padding) / options.resolution) *
      options.resolution;
  const double maximum_x = bounds.maximum.x + options.padding;
  const double maximum_y = bounds.maximum.y + options.padding;
  const double columns = std::ceil((maximum_x - origin_x) / options.resolution);
  const double rows = std::ceil((maximum_y - origin_y) / options.resolution);
  if (!std::isfinite(columns) || !std::isfinite(rows) || columns <= 0 || rows <= 0 ||
      columns * rows > kMaximumCells)
    throw std::runtime_error("map too large; check units and resolution");
  const int width =
      static_cast<int>(std::ceil((maximum_x - origin_x) / options.resolution));
  const int height =
      static_cast<int>(std::ceil((maximum_y - origin_y) / options.resolution));
  if (width <= 0 || height <= 0 ||
      static_cast<unsigned long long>(width) *
              static_cast<unsigned long long>(height) >
          kMaximumCells) {
    throw std::runtime_error(
        "generated map dimensions are invalid or exceed 100 million cells; "
        "check scale and resolution");
  }

  OccupancyMap map;
  map.width = width;
  map.height = height;
  map.resolution = options.resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;
  map.occupied.assign(static_cast<std::size_t>(width) * height, 0);

  OccupancyMap support = map;

  std::vector<double> elevation(map.occupied.size(), -std::numeric_limits<double>::infinity());
  if(options.terrain_mode) {
    const double min_normal=std::cos(options.max_slope_degrees*3.141592653589793/180.0);
    for(const auto &raw : mesh.triangles()) {
      auto t=scaled(raw,options.scale);
      if(std::abs(triangleNormal(t).z)<min_normal) continue;
      if(std::max({t.a.z,t.b.z,t.c.z}) < -options.ground_tolerance ||
         std::min({t.a.z,t.b.z,t.c.z}) > options.max_support_z) continue;
      visitSurface(map,t,[&](std::size_t i,double lo,double hi) {
        if(lo>=-options.ground_tolerance && hi<=options.max_support_z)
          elevation[i]=std::max(elevation[i],hi);
      });
    }
    for(std::size_t i=0;i<elevation.size();++i)
      support.occupied[i]=std::isfinite(elevation[i])?1:0;
  }

  std::size_t selected = 0;
  for (const Triangle &raw_triangle : mesh.triangles()) {
    const Triangle triangle = scaled(raw_triangle, options.scale);
    const double minimum_z =
        std::min({triangle.a.z, triangle.b.z, triangle.c.z});
    const double maximum_z =
        std::max({triangle.a.z, triangle.b.z, triangle.c.z});
    const Vec3 normal = triangleNormal(triangle);
    if(options.terrain_mode) {
      if(maximum_z < -options.ground_tolerance || minimum_z > options.max_support_z+options.robot_height)
        continue;
      bool intersects=false;
      visitSurface(map,triangle,[&](std::size_t i,double lo,double hi) {
        if(support.occupied[i] && hi > elevation[i]+options.max_step_height+1e-6 &&
           lo <= elevation[i]+options.robot_height+1e-6) {
          map.occupied[i]=1;intersects=true;
        }
      });
      if(intersects) ++selected;
      continue;
    }
    if (options.require_ground && std::abs(normal.z) > 0.99 &&
        minimum_z >= -options.ground_tolerance && maximum_z <= options.ground_tolerance) {
      rasterizeTriangle(support, triangle);
    }
    if (maximum_z < options.slice_min_z || minimum_z > options.slice_max_z) {
      continue;
    }
    if (std::abs(normal.z) > options.max_abs_normal_z) {
      continue;
    }
    ++selected;
    auto polygon = clipHeight({triangle.a, triangle.b, triangle.c}, options.slice_min_z, true);
    polygon = clipHeight(polygon, options.slice_max_z, false);
    for (std::size_t i = 1; i + 1 < polygon.size(); ++i)
      rasterizeTriangle(map, {polygon[0], polygon[i], polygon[i + 1]});
  }

  if (selected == 0 && !options.terrain_mode) {
    throw std::runtime_error(
        "no triangles matched the height/normal filters; inspect STL units and "
        "slice parameters");
  }

  if (options.require_ground || options.terrain_mode) {
    if (support.occupiedCount() == 0) throw std::runtime_error("no ground support at z=0");
    for (std::size_t i = 0; i < map.occupied.size(); ++i)
      if (!support.occupied[i]) map.occupied[i] = 1;
  }
  if(options.terrain_mode) {
    for(int y=0;y<map.height;++y) for(int x=0;x<map.width;++x) {
      auto i=cellIndex(map,x,y);
      if(!support.occupied[i]) continue;
      for(auto d : {std::pair<int,int>{1,0},{-1,0},{0,1},{0,-1}}) {
        int nx=x+d.first,ny=y+d.second;
        if(!map.inBounds(nx,ny)) {map.occupied[i]=1;continue;}
        auto j=cellIndex(map,nx,ny);
        if(!support.occupied[j] || std::abs(elevation[i]-elevation[j])>options.max_step_height)
          map.occupied[i]=1;
      }
    }
  }
  const std::size_t raw_occupied = map.occupiedCount();
  inflate(map, options.robot_radius);

  // Ground-only mode represents the main connected driving area. Sealed hollow
  // objects and disconnected elevated surfaces must not appear as free islands.
  if (options.require_ground || options.terrain_mode) {
    std::vector<std::uint8_t> visited(map.occupied.size(), 0);
    std::vector<std::size_t> largest;
    for (std::size_t seed = 0; seed < visited.size(); ++seed) {
      if (visited[seed] || map.occupied[seed]) continue;
      std::vector<std::size_t> component{seed};
      visited[seed] = 1;
      for (std::size_t q = 0; q < component.size(); ++q) {
        int x = static_cast<int>(component[q] % map.width);
        int y = static_cast<int>(component[q] / map.width);
        for (auto d : {std::pair<int,int>{1,0}, {-1,0}, {0,1}, {0,-1}}) {
          int nx = x + d.first, ny = y + d.second;
          if (!map.inBounds(nx,ny)) continue;
          auto next = cellIndex(map,nx,ny);
          if (!visited[next] && !map.occupied[next]) {
            visited[next] = 1;
            component.push_back(next);
          }
        }
      }
      if (component.size() > largest.size()) largest = std::move(component);
    }
    if (largest.empty()) throw std::runtime_error("no traversable ground remains");
    std::fill(map.occupied.begin(), map.occupied.end(), std::uint8_t{1});
    for (auto index : largest) map.occupied[index] = 0;
  }

  return ConversionResult{map,
                          mesh.triangles().size(),
                          selected,
                          raw_occupied,
                          map.occupiedCount(),
                          bounds};
}

void writePgmAndYaml(const OccupancyMap &map,
                     const std::string &output_prefix) {
  if (map.width <= 0 || map.height <= 0 ||
      map.occupied.size() !=
          static_cast<std::size_t>(map.width) * map.height) {
    throw std::invalid_argument("occupancy map is invalid");
  }

  const std::filesystem::path prefix(output_prefix);
  if (prefix.has_parent_path()) {
    std::filesystem::create_directories(prefix.parent_path());
  }
  const std::filesystem::path pgm_path = prefix.string() + ".pgm";
  const std::filesystem::path yaml_path = prefix.string() + ".yaml";

  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("cannot create PGM file: " + pgm_path.string());
  }
  pgm << "P5\n" << map.width << ' ' << map.height << "\n255\n";
  for (int y = map.height - 1; y >= 0; --y) {
    for (int x = 0; x < map.width; ++x) {
      const unsigned char pixel = map.isOccupied(x, y) ? 0 : 254;
      pgm.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }

  std::ofstream yaml(yaml_path);
  if (!yaml) {
    throw std::runtime_error("cannot create YAML file: " + yaml_path.string());
  }
  yaml << "image: " << pgm_path.filename().string() << '\n';
  yaml << "mode: trinary\n";
  yaml << "resolution: " << map.resolution << '\n';
  yaml << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0.0]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: 0.65\n";
  yaml << "free_thresh: 0.25\n";
}

}  // namespace stl_to_grid_map
