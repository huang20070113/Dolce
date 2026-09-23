#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "stl_to_grid_map/grid_converter.hpp"
#include "stl_to_grid_map/stl_mesh.hpp"

namespace stl_to_grid_map {

class StlToGridNode : public rclcpp::Node {
public:
  StlToGridNode() : Node("stl_to_grid_map") {
    const std::string input_file =
        declare_parameter<std::string>("input_file", "RMUC2025.STL");
    const std::string output_prefix =
        declare_parameter<std::string>("output_prefix", "output/RMUC2025");//读取 STL 输入文件路径
    const std::string frame_id = declare_parameter<std::string>("frame_id", "map");//设置地图坐标系名称<map>

    ConversionOptions options;
    options.resolution = declare_parameter<double>("resolution", 0.05);
    options.robot_radius = declare_parameter<double>("robot_radius", 0.25);
    options.slice_min_z = declare_parameter<double>("slice_min_z", 0.10);
    options.slice_max_z = declare_parameter<double>("slice_max_z", 0.60);
    options.scale = declare_parameter<double>("scale", 0.001);
    options.padding = declare_parameter<double>("padding", 0.50);
    options.max_abs_normal_z =
        declare_parameter<double>("max_abs_normal_z", 1.0);
    options.require_ground = declare_parameter<bool>("require_ground", true);
    options.ground_tolerance = declare_parameter<double>("ground_tolerance", 0.03);
    options.terrain_mode = declare_parameter<bool>("terrain_mode", true);
    options.max_support_z = declare_parameter<double>("max_support_z", 0.65);
    options.max_slope_degrees = declare_parameter<double>("max_slope_degrees", 25.0);
    options.max_step_height = declare_parameter<double>("max_step_height", 0.06);
    options.robot_height = declare_parameter<double>("robot_height", 0.60);

    const auto mesh = StlMesh::load(input_file);
    const auto result = convertMesh(mesh, options);
    writePgmAndYaml(result.map, output_prefix);

    message_.header.frame_id = frame_id;
    message_.info.resolution = static_cast<float>(result.map.resolution);
    message_.info.width = static_cast<std::uint32_t>(result.map.width);
    message_.info.height = static_cast<std::uint32_t>(result.map.height);
    message_.info.origin.position.x = result.map.origin_x;
    message_.info.origin.position.y = result.map.origin_y;
    message_.info.origin.orientation.w = 1.0;
    message_.data.resize(result.map.occupied.size());
    for (std::size_t index = 0; index < result.map.occupied.size(); ++index) {
      message_.data[index] = result.map.occupied[index] ? 100 : 0;
    }

    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local().reliable());
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      message_.header.stamp = now();
      publisher_->publish(message_);
    });

    RCLCPP_INFO(get_logger(),
                "Generated %dx%d map: %zu/%zu triangles selected, %zu cells "
                "occupied after inflation",
                result.map.width, result.map.height, result.selected_triangles,
                result.total_triangles, result.inflated_occupied_cells);
  }

private:
  nav_msgs::msg::OccupancyGrid message_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace stl_to_grid_map

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);//初始化 ROS2
  try {
    rclcpp::spin(std::make_shared<stl_to_grid_map::StlToGridNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("stl_to_grid_map"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
