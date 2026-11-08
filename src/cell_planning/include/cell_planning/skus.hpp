#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <cell_planning/grasps.hpp>      // BoxSize

namespace cell_planning {

/// The SKU table (what a WMS would provide).
inline const std::map<std::string, BoxSize> SKUS{{"A", {0.30, 0.18, 0.15}}, {"B", {0.20, 0.20, 0.12}}, {"C", {0.40, 0.30, 0.20}}};

/// Footprint lookup (the detector gives the long side first). Empty name = no match: measured footprint, default height.
inline std::pair<std::string, BoxSize> matchSku(double sx, double sy, double tol = 0.03) {
  for (const auto & [name, b] : SKUS)
    if (std::abs(sx - std::max(b.x, b.y)) <= tol && std::abs(sy - std::min(b.x, b.y)) <= tol) return {name, b};
  return {"", BoxSize{sx, sy, 0.15}};
}

}  // namespace cell_planning
