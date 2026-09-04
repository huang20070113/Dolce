#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <vector>

enum class SensorType { CAMERA, IMU };

struct SensorData {
  SensorType type;
  std::uint32_t sequence;
  long long timestamp_us;
};

struct SyncResult {
  SensorData camera;
  std::optional<SensorData> imu;
  long long delta_us = 0;
};

inline std::vector<SyncResult> synchronize(
    std::vector<SensorData> cameras,
    std::vector<SensorData> imus,
    long long max_delta_us) {
  if (max_delta_us < 0) {
    throw std::invalid_argument("max_delta_us must not be negative");
  }

  const auto packet_less = [](const SensorData& lhs, const SensorData& rhs) {
    if (lhs.timestamp_us != rhs.timestamp_us) {
      return lhs.timestamp_us < rhs.timestamp_us;
    }
    return lhs.sequence < rhs.sequence;
  };
  std::sort(cameras.begin(), cameras.end(), packet_less);
  std::sort(imus.begin(), imus.end(), packet_less);

  std::vector<SyncResult> results;
  results.reserve(cameras.size());
  // Both vectors are sorted. The IMU cursor only moves forward, making the
  // matching pass linear after the initial sorting.
  std::size_t next_imu = 0;
  for (const auto& camera : cameras) {
    SyncResult result{camera, std::nullopt, 0};

    const SensorData* best = nullptr;
    const auto consider = [&](const SensorData& candidate) {
      const long long delta = candidate.timestamp_us >= camera.timestamp_us
                                  ? candidate.timestamp_us - camera.timestamp_us
                                  : camera.timestamp_us - candidate.timestamp_us;
      if (best == nullptr) {
        best = &candidate;
        result.delta_us = delta;
        return;
      }
      if (delta < result.delta_us ||
          (delta == result.delta_us &&
           (candidate.timestamp_us < best->timestamp_us ||
            (candidate.timestamp_us == best->timestamp_us &&
             candidate.sequence < best->sequence)))) {
        best = &candidate;
        result.delta_us = delta;
      }
    };

    while (next_imu < imus.size() &&
           imus[next_imu].timestamp_us < camera.timestamp_us) {
      ++next_imu;
    }
    if (next_imu < imus.size()) {
      consider(imus[next_imu]);
    }
    if (next_imu != 0) {
      consider(imus[next_imu - 1]);
    }

    if (best != nullptr && result.delta_us <= max_delta_us) {
      result.imu = *best;
    } else {
      result.delta_us = 0;
    }
    results.push_back(result);
  }
  return results;
}
