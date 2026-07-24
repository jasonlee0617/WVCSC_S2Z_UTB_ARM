#pragma once

#include <cstddef>
#include <cmath>
#include <limits>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace wtb_car_driver
{

struct SelfMaskBounds
{
  double min_x{-0.825};
  double max_x{0.825};
  double min_y{-0.60};
  double max_y{0.60};
};

inline bool isInsideSelfMask(double x, double y, const SelfMaskBounds & bounds)
{
  return x >= bounds.min_x && x <= bounds.max_x &&
         y >= bounds.min_y && y <= bounds.max_y;
}

inline void maskSelfReturns(
  sensor_msgs::msg::LaserScan & scan,
  double base_from_scan_x,
  double base_from_scan_y,
  double base_from_scan_yaw,
  const SelfMaskBounds & bounds)
{
  const double cosine = std::cos(base_from_scan_yaw);
  const double sine = std::sin(base_from_scan_yaw);
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!std::isfinite(range)) {
      continue;
    }
    const double angle = scan.angle_min + index * scan.angle_increment;
    const double scan_x = range * std::cos(angle);
    const double scan_y = range * std::sin(angle);
    const double base_x = base_from_scan_x + cosine * scan_x - sine * scan_y;
    const double base_y = base_from_scan_y + sine * scan_x + cosine * scan_y;
    if (isInsideSelfMask(base_x, base_y, bounds)) {
      // NaN is deliberately ignored by Nav2's obstacle layer.  Infinity
      // would falsely clear unknown space behind a self-occluding object.
      scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

}  // namespace wtb_car_driver
