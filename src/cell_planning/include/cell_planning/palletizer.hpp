#pragma once
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <cell_planning/patterns.hpp>

namespace cell_planning {

using Placed = std::vector<std::pair<Isometry3d, BoxSize>>;     // belief: what sits on the pallet

/// What the executive needs from any placement plan: a target for the next box, confirmed once it is placed.
struct Plan {
  virtual ~Plan() = default;
  virtual std::optional<Isometry3d> target(const BoxSize & size) = 0;   // box center in pallet_frame, or nothing when full
  virtual void confirm() = 0;
  virtual const Placed & placed() const = 0;
};

/// Cursor over an interlocked single-SKU stack. target() is retried until confirm() advances it.
class SingleSkuPlan : public Plan {
public:
  explicit SingleSkuPlan(int layers = 3, const BoxSize & box = SKU_A) : box_(box) {
    int L = 0;
    for (const auto & layer : stack(layers)) {
      for (const auto & p : placeOrder(layer)) targets_.emplace_back(L, p);
      ++L;
    }
  }
  std::optional<Isometry3d> target(const BoxSize &) override {          // size ignored: every box is SKU_A
    if (i_ >= targets_.size()) return std::nullopt;
    return boxPose(targets_[i_].second, targets_[i_].first, box_);
  }
  void confirm() override { placed_.emplace_back(*target(box_), box_); ++i_; }
  const Placed & placed() const override { return placed_; }

private:
  BoxSize box_;
  std::vector<std::pair<int, Place>> targets_;
  std::size_t i_ = 0;
  Placed placed_;
};

}  // namespace cell_planning
