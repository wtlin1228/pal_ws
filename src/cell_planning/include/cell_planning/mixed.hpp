#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>
#include <utility>
#include <Eigen/Core>
#include <cell_planning/palletizer.hpp>

namespace cell_planning {

/// Belief of the pallet surface in pallet_frame, 1 cm cells: the same structure the detector builds (Ch 10).
class HeightMap {
public:
  struct Window { int x0, y0, x1, y1; };                           // cells [x0, x1) x [y0, y1)

  explicit HeightMap(double cell = 0.01, double gap = 0.01)
  : c_(cell), gap_(gap), nx_(std::lround(PALLET[0] / cell)), ny_(std::lround(PALLET[1] / cell)),
    h_(Eigen::MatrixXd::Zero(ny_, nx_)), halo_(Eigen::MatrixXd::Zero(ny_, nx_)) {}

  /// Cells covering a w x d footprint centered at (x, y). Off the deck: nothing, or clipped if clip is set.
  std::optional<Window> window(double x, double y, double w, double d, bool clip = false) const {
    int x0 = std::lround((x - w / 2 + PALLET[0] / 2) / c_), x1 = x0 + std::lround(w / c_);
    int y0 = std::lround((y - d / 2 + PALLET[1] / 2) / c_), y1 = y0 + std::lround(d / c_);
    if (x0 < 0 || y0 < 0 || x1 > nx_ || y1 > ny_) {
      if (!clip) return std::nullopt;
      x0 = std::max(x0, 0); y0 = std::max(y0, 0); x1 = std::min(x1, nx_); y1 = std::min(y1, ny_);
    }
    return Window{x0, y0, x1, y1};
  }

  /// Height the box would rest at, and the fraction of its footprint touching that height.
  /// The halo counts for the rest height but never as support: a box touching a neighbor is unsupported.
  std::pair<double, double> rest(const Window & w, double tol = 0.005) const {
    const auto H = h_.block(w.y0, w.x0, w.y1 - w.y0, w.x1 - w.x0);
    const double z = std::max(H.maxCoeff(), halo_.block(w.y0, w.x0, w.y1 - w.y0, w.x1 - w.x0).maxCoeff());
    const double support = (H.array() >= z - tol).cast<double>().mean();
    return {z, support};
  }

  void place(double x, double y, double w, double d, double z_top) {
    const Window win = *window(x, y, w, d);
    h_.block(win.y0, win.x0, win.y1 - win.y0, win.x1 - win.x0).setConstant(z_top);
    const Window g = *window(x, y, w + 2 * gap_, d + 2 * gap_, true);   // keep-out ring around every placed box
    auto halo = halo_.block(g.y0, g.x0, g.y1 - g.y0, g.x1 - g.x0);
    halo = halo.cwiseMax(z_top);
  }

  const Eigen::MatrixXd & heights() const { return h_; }

private:
  double c_, gap_;
  int nx_, ny_;
  Eigen::MatrixXd h_, halo_;
};

struct Placement { double x, y, yaw, z; };

/// Lowest rest height, then most support, then the far-left corner.
inline std::optional<Placement> bestPlacement(const HeightMap & hm, const BoxSize & size, double step = 0.02,
                                              double min_support = 0.7, double max_height = 0.6) {
  std::optional<Placement> best;
  std::tuple<double, double, double, double> best_key;
  for (double yaw : {0.0, M_PI / 2}) {
    const auto [w, d] = footprint(size, yaw);
    for (double x = -PALLET[0] / 2 + w / 2; x <= PALLET[0] / 2 - w / 2 + 1e-9; x += step)
      for (double y = -PALLET[1] / 2 + d / 2; y <= PALLET[1] / 2 - d / 2 + 1e-9; y += step) {
        const auto win = hm.window(x, y, w, d);
        if (!win) continue;
        const auto [z, support] = hm.rest(*win);
        if (support < min_support || z + size.z > max_height) continue;
        const auto key = std::make_tuple(std::round(z * 1000) / 1000, -std::round(support * 100) / 100,
                                         -std::round(y * 1000) / 1000, std::round(x * 1000) / 1000);   // low, supported, far side, left
        if (!best || key < best_key) { best = Placement{x, y, yaw, z}; best_key = key; }
      }
  }
  return best;
}

/// Online placement, one box at a time; sizes known on arrival. Same Plan interface as SingleSkuPlan.
class MixedPlan : public Plan {
public:
  explicit MixedPlan(double max_height = 0.6) : max_height_(max_height) {}

  std::optional<Isometry3d> target(const BoxSize & size) override {
    const auto p = bestPlacement(hm_, size, 0.02, 0.7, max_height_);
    if (!p) return std::nullopt;
    pending_ = {*p, size};
    return trans(p->x, p->y, p->z + size.z / 2) * rotZ(p->yaw);
  }
  void confirm() override {
    const auto & [p, size] = pending_;
    const auto [w, d] = footprint(size, p.yaw);
    hm_.place(p.x, p.y, w, d, p.z + size.z);
    placed_.emplace_back(trans(p.x, p.y, p.z + size.z / 2) * rotZ(p.yaw), size);
  }
  const Placed & placed() const override { return placed_; }
  const HeightMap & heightMap() const { return hm_; }

private:
  HeightMap hm_;
  double max_height_;
  std::pair<Placement, BoxSize> pending_{};
  Placed placed_;
};

}  // namespace cell_planning
