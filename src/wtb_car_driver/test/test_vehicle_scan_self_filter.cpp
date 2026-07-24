#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "wtb_car_driver/vehicle_scan_self_filter.hpp"

namespace
{

sensor_msgs::msg::LaserScan makeScan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0F;
  scan.angle_increment = 1.5707963267948966F;
  scan.ranges = {0.20F, 0.80F, std::numeric_limits<float>::infinity()};
  return scan;
}

TEST(VehicleScanSelfFilter, MasksOnlyReturnsInsideBaseFootprint)
{
  auto scan = makeScan();
  // laser is at x=0.4 in base_footprint. The first point is x=0.6 and is
  // inside the 0.825 m front self-mask; the second point is y=0.8 and remains.
  wtb_car_driver::maskSelfReturns(scan, 0.4, 0.0, 0.0, {});

  EXPECT_TRUE(std::isnan(scan.ranges[0]));
  EXPECT_FLOAT_EQ(scan.ranges[1], 0.80F);
  EXPECT_TRUE(std::isinf(scan.ranges[2]));
}

TEST(VehicleScanSelfFilter, PreservesAnObstacleOutsideTheMask)
{
  auto scan = makeScan();
  scan.ranges[0] = 0.50F;  // x=0.9 in base_footprint, outside the mask.
  wtb_car_driver::maskSelfReturns(scan, 0.4, 0.0, 0.0, {});

  EXPECT_FLOAT_EQ(scan.ranges[0], 0.50F);
}

}  // namespace
