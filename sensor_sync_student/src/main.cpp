#include <fstream>
#include <exception>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "sensor_sync.hpp"
#include "thread_safe_queue.hpp"

int main(int argc, char* argv[]) {
  try {
    if (argc != 2) {
      throw std::invalid_argument("usage: sensor_sync <config-file>");
    }

    const std::string config_path = argv[1];
    std::ifstream config(config_path);
    if (!config) {
      throw std::runtime_error("cannot open configuration: " + config_path);
    }

    auto trim = [](std::string value) {
      const auto first = value.find_first_not_of(" \t\r\n");
      if (first == std::string::npos) return std::string{};
      const auto last = value.find_last_not_of(" \t\r\n");
      return value.substr(first, last - first + 1);
    };
    std::unordered_map<std::string, std::string> settings;
    std::string line;
    while (std::getline(config, line)) {
      line = trim(line);
      if (line.empty() || line.front() == '#') continue;
      const auto equal = line.find('=');
      if (equal == std::string::npos) {
        throw std::runtime_error("malformed configuration line: " + line);
      }
      const std::string key = trim(line.substr(0, equal));
      const std::string value = trim(line.substr(equal + 1));
      if (key.empty() || value.empty() || settings.count(key) != 0) {
        throw std::runtime_error("invalid or duplicate configuration key: " + key);
      }
      settings.emplace(key, value);
    }

    const std::vector<std::string> required = {
        "queue_capacity", "max_delta_us", "camera_file", "imu_file", "output_file"};
    for (const auto& key : required) {
      if (!settings.count(key)) throw std::runtime_error("missing configuration key: " + key);
    }
    const auto parse_positive = [&](const std::string& key, bool allow_zero) {
      std::size_t used = 0;
      const auto value = std::stoll(settings.at(key), &used);
      if (used != settings.at(key).size() || value < 0 || (!allow_zero && value == 0)) {
        throw std::runtime_error("invalid numeric value for " + key);
      }
      return value;
    };
    const auto queue_capacity = static_cast<std::size_t>(parse_positive("queue_capacity", false));
    const auto max_delta_us = parse_positive("max_delta_us", true);

    const auto base = std::string(config_path).find_last_of("/\\");
    const std::string base_dir = base == std::string::npos ? "." : config_path.substr(0, base);
    const auto resolve = [&](const std::string& path) {
      if (path.empty() || path.front() == '/' || (path.size() > 1 && path[1] == ':')) return path;
      return base_dir + "/" + path;
    };

    ThreadSafeQueue<SensorData> queue(queue_capacity);
    std::vector<SensorData> cameras;
    std::vector<SensorData> imus;
    std::mutex result_mutex;
    std::exception_ptr thread_error;
    std::mutex error_mutex;

    const auto read_file = [&](const std::string& path, SensorType type) {
      std::ifstream input(path);
      if (!input) throw std::runtime_error("cannot open data file: " + path);
      std::string record;
      std::size_t line_number = 0;
      while (std::getline(input, record)) {
        ++line_number;
        if (record.empty()) throw std::runtime_error("empty record in " + path);
        std::istringstream parser(record);
        long long sequence_value = -1;
        long long timestamp = -1;
        std::string extra;
        if (!(parser >> sequence_value >> timestamp) || (parser >> extra) ||
            sequence_value < 0 || sequence_value > std::numeric_limits<std::uint32_t>::max() ||
            timestamp < 0) {
          throw std::runtime_error("malformed record in " + path + " at line " + std::to_string(line_number));
        }
        if (!queue.push(SensorData{type, static_cast<std::uint32_t>(sequence_value), timestamp})) return;
      }
    };

    const auto producer = [&](const std::string& path, SensorType type) {
      try {
        read_file(path, type);
      } catch (...) {
        std::lock_guard<std::mutex> lock(error_mutex);
        if (!thread_error) thread_error = std::current_exception();
        queue.close();
      }
    };
    std::thread camera_thread(producer, resolve(settings.at("camera_file")), SensorType::CAMERA);
    std::thread imu_thread(producer, resolve(settings.at("imu_file")), SensorType::IMU);
    std::thread consumer([&] {
      try {
        SensorData packet{};
        while (queue.pop(packet)) {
          std::lock_guard<std::mutex> lock(result_mutex);
          (packet.type == SensorType::CAMERA ? cameras : imus).push_back(packet);
        }
      } catch (...) {
        std::lock_guard<std::mutex> lock(error_mutex);
        if (!thread_error) thread_error = std::current_exception();
        queue.close();
      }
    });

    camera_thread.join();
    imu_thread.join();
    queue.close();
    consumer.join();
    {
      std::lock_guard<std::mutex> lock(error_mutex);
      if (thread_error) std::rethrow_exception(thread_error);
    }

    const auto results = synchronize(std::move(cameras), std::move(imus), max_delta_us);
    std::ofstream output(resolve(settings.at("output_file")), std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open output file: " + resolve(settings.at("output_file")));
    for (const auto& result : results) {
      output << "CAMERA " << result.camera.sequence;
      if (result.imu) {
        output << " IMU " << result.imu->sequence << " DELTA " << result.delta_us;
      } else {
        output << " UNMATCHED";
      }
      output << '\n';
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "sensor_sync: " << error.what() << '\n';
    return 1;
  }
}
