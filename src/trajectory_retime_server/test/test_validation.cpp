#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "trajectory_retime_server/validation.hpp"

namespace validation = trajectory_retime_server::validation;

TEST(Validation, ScalingMustBeFiniteAndInsideOpenClosedUnitInterval)
{
  EXPECT_TRUE(validation::valid_scaling(0.1));
  EXPECT_TRUE(validation::valid_scaling(1.0));
  EXPECT_FALSE(validation::valid_scaling(0.0));
  EXPECT_FALSE(validation::valid_scaling(-0.1));
  EXPECT_FALSE(validation::valid_scaling(1.0001));
  EXPECT_FALSE(validation::valid_scaling(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(validation::valid_scaling(std::numeric_limits<double>::quiet_NaN()));
}

TEST(Validation, JointNamesMustBeNonemptyAndUnique)
{
  EXPECT_TRUE(validation::unique_nonempty_names({"joint1", "joint2"}));
  EXPECT_FALSE(validation::unique_nonempty_names({"joint1", "joint1"}));
  EXPECT_FALSE(validation::unique_nonempty_names({"joint1", ""}));
}
