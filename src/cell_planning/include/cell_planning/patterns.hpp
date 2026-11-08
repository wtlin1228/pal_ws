#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>
#include <cell_planning/geometry.hpp>
#include <cell_planning/grasps.hpp>      // BoxSize

namespace cell_planning {

// Layer patterns in pallet_frame: x along the 0.8 m side, y along the 0.6 m side, origin at the deck center.
constexpr std::array<double, 2> PALLET = {0.80, 0.60};     // x, y [m]
constexpr BoxSize SKU_A{0.30, 0.18, 0.15};                  // x, y, z [m]
constexpr double GAP = 0.02;

struct Place { double x, y, yaw; };   // box center in pallet_frame; yaw 0: box x along pallet x, pi/2: box x along pallet y
struct Row { int n; double yaw; };

/// Extent along pallet x and y for yaw 0 or pi/2.
inline std::pair<double, double> footprint(const BoxSize & box, double yaw) {
  return std::abs(yaw) < 1e-9 ? std::pair<double, double>{box.x, box.y} : std::pair<double, double>{box.y, box.x};
}

/// n boxes side by side along x, centered, all at row center y.
inline std::vector<Place> row(int n, double yaw, double y, const BoxSize & box = SKU_A, double gap = GAP) {
  const auto [w, d] = footprint(box, yaw);
  const double x0 = -(n * w + (n - 1) * gap) / 2;
  std::vector<Place> out;
  for (int i = 0; i < n; ++i) out.push_back({x0 + w / 2 + i * (w + gap), y, yaw});
  return out;
}

/// Rows stacked along y, the whole block centered on the deck.
inline std::vector<Place> layerFromRows(const std::vector<Row> & rows, const BoxSize & box = SKU_A, double gap = GAP) {
  double total = gap * (rows.size() - 1);
  for (const auto & r : rows) total += footprint(box, r.yaw).second;
  double y = -total / 2;
  std::vector<Place> places;
  for (const auto & r : rows) {
    const double d = footprint(box, r.yaw).second;
    for (const auto & p : row(r.n, r.yaw, y + d / 2, box, gap)) places.push_back(p);
    y += d + gap;
  }
  return places;
}

inline bool fits(const std::vector<Place> & places, const BoxSize & box = SKU_A, double overhang = 0.0) {
  for (const auto & p : places) {
    const auto [w, d] = footprint(box, p.yaw);
    if (std::abs(p.x) + w / 2 > PALLET[0] / 2 + overhang + 1e-9 || std::abs(p.y) + d / 2 > PALLET[1] / 2 + overhang + 1e-9) return false;
  }
  return true;
}

/// True if any two footprints intersect (yaw 0 / pi/2 only).
inline bool overlaps(const std::vector<Place> & places, const BoxSize & box = SKU_A) {
  for (std::size_t i = 0; i < places.size(); ++i) {
    const auto [wa, da] = footprint(box, places[i].yaw);
    for (std::size_t j = i + 1; j < places.size(); ++j) {
      const auto [wb, db] = footprint(box, places[j].yaw);
      if (std::abs(places[i].x - places[j].x) < (wa + wb) / 2 - 1e-9 && std::abs(places[i].y - places[j].y) < (da + db) / 2 - 1e-9) return true;
    }
  }
  return false;
}

inline double utilization(const std::vector<Place> & places, const BoxSize & box = SKU_A) {
  return places.size() * box.x * box.y / (PALLET[0] * PALLET[1]);
}

inline const std::vector<Place> & layerA() { static const auto L = layerFromRows({{2, 0.0}, {2, 0.0}, {2, 0.0}}); return L; }   // 2 x 3 grid, long side along x
inline const std::vector<Place> & layerB() { static const auto L = layerFromRows({{4, M_PI / 2}, {2, 0.0}}); return L; }       // 4 across + 2 along: the seams move

/// Alternate A and B so the vertical seams of one layer land on box interiors of the next.
inline std::vector<std::vector<Place>> stack(int layers) {
  std::vector<std::vector<Place>> s;
  for (int L = 0; L < layers; ++L) s.push_back(L % 2 ? layerB() : layerA());
  return s;
}

/// Far side first: the robot stands at negative y of the pallet, so place large y first.
inline std::vector<Place> placeOrder(std::vector<Place> layer) {
  std::sort(layer.begin(), layer.end(), [](const Place & a, const Place & b) { return a.y != b.y ? a.y > b.y : a.x < b.x; });
  return layer;
}

/// Pose of the box center in pallet_frame; layers rest directly on each other.
inline Isometry3d boxPose(const Place & p, int layer_index, const BoxSize & box = SKU_A) {
  return trans(p.x, p.y, layer_index * box.z + box.z / 2) * rotZ(p.yaw);
}

/// Ch 12 checkpoint: fraction of layer b's seam length that lies over a box interior of layer a.
inline double seamOverlap(const std::vector<Place> & a, const std::vector<Place> & b, const BoxSize & box = SKU_A, double step = 0.005) {
  auto interior = [&](double x, double y) {
    for (const auto & p : a) {
      const auto [w, d] = footprint(box, p.yaw);
      if (std::abs(x - p.x) < w / 2 - 0.01 && std::abs(y - p.y) < d / 2 - 0.01) return true;
    }
    return false;
  };
  double hits = 0, total = 0;
  for (const auto & p : b) {
    const auto [w, d] = footprint(box, p.yaw);
    for (double s = -w / 2; s < w / 2; s += step) for (double yy : {p.y - d / 2, p.y + d / 2}) { ++total; hits += interior(p.x + s, yy); }
    for (double s = -d / 2; s < d / 2; s += step) for (double xx : {p.x - w / 2, p.x + w / 2}) { ++total; hits += interior(xx, p.y + s); }
  }
  return hits / total;
}

}  // namespace cell_planning
