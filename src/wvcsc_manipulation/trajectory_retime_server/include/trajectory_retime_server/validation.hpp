// 中文说明：轨迹重定时请求的纯参数校验辅助声明。
// 校验只保证缩放因子和关节轨迹输入满足服务契约，不执行运动或改变原始轨迹语义。
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
