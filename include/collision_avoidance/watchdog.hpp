#pragma once
#include <cmath>
namespace collision_avoidance {
inline bool inputsFresh(double command_age, double scan_age, bool scan_valid,
                        double command_timeout, double scan_timeout) {
  return scan_valid && std::isfinite(command_age) && std::isfinite(scan_age) &&
      command_age >= 0.0 && scan_age >= 0.0 &&
      command_age <= command_timeout && scan_age <= scan_timeout;
}
}
