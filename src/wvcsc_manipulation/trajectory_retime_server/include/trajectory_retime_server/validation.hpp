#ifndef TRAJECTORY_RETIME_SERVER__VALIDATION_HPP_
#define TRAJECTORY_RETIME_SERVER__VALIDATION_HPP_

#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace trajectory_retime_server::validation
{

inline bool valid_scaling(const double value)
{
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

inline bool unique_nonempty_names(const std::vector<std::string> & names)
{
  std::unordered_set<std::string> seen;
  seen.reserve(names.size());
  for (const auto & name : names) {
    if (name.empty() || !seen.insert(name).second) {
      return false;
    }
  }
  return true;
}

}  // namespace trajectory_retime_server::validation

#endif  // TRAJECTORY_RETIME_SERVER__VALIDATION_HPP_
