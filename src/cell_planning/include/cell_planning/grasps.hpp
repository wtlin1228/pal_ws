#pragma once
#include <array>
#include <cmath>
#include <utility>
#include <vector>
#include <cell_planning/geometry.hpp>

namespace cell_planning {

struct BoxSize { double x, y, z; };
using Neighbor = std::pair<Isometry3d, BoxSize>;                 // (pose of the box center, its size)
constexpr std::array<double, 2> PAD = {0.25, 0.15};              // suction pad footprint x, y [m] (Ch 3)

/// Top-down TCP poses for a box: pad along the long side, both yaws, TCP z into the box.
inline std::vector<Isometry3d> graspCandidates(const Isometry3d & T_box, const BoxSize & size) {
  const Isometry3d T_top = T_box * trans(0.0, 0.0, size.z / 2);
  const double align = size.x >= size.y ? 0.0 : M_PI / 2;
  return {T_top * rotZ(align) * rotX(M_PI), T_top * rotZ(align + M_PI) * rotX(M_PI)};
}

/// Sample points of the pad face, expressed in the frame T_tcp is given in.
inline std::vector<Eigen::Vector3d> padPoints(const Isometry3d & T_tcp, double step = 0.01) {
  std::vector<Eigen::Vector3d> P;
  for (double x = -PAD[0] / 2; x <= PAD[0] / 2 + 1e-9; x += step)
    for (double y = -PAD[1] / 2; y <= PAD[1] / 2 + 1e-9; y += step) P.push_back(T_tcp * Eigen::Vector3d(x, y, 0.0));
  return P;
}

/// Does a point lie on the top face of a box? (tested in the box's own frame)
inline bool insideTop(const Eigen::Vector3d & p, const Isometry3d & T_box, const BoxSize & s, double z_tol = 0.02) {
  const Eigen::Vector3d l = T_box.inverse() * p;
  return std::abs(l.x()) <= s.x / 2 && std::abs(l.y()) <= s.y / 2 && std::abs(l.z() - s.z / 2) <= z_tol;
}

/// Fraction of the pad resting on other boxes whose top is at the pad height.
inline double doublePickRisk(const Isometry3d & T_tcp, const std::vector<Neighbor> & neighbors) {
  const auto P = padPoints(T_tcp);
  std::size_t hits = 0;
  for (const auto & p : P)
    for (const auto & [T_n, s_n] : neighbors)
      if (insideTop(p, T_n, s_n)) { ++hits; break; }
  return static_cast<double>(hits) / static_cast<double>(P.size());
}

/// (risk, T_tcp): lowest double-pick risk, then least wrist rotation from the current TCP if one is given.
inline std::pair<double, Isometry3d> bestGrasp(const Isometry3d & T_box, const BoxSize & size,
                                               const std::vector<Neighbor> & neighbors,
                                               const Isometry3d * T_tcp_now = nullptr) {
  std::pair<double, Isometry3d> best{2.0, Isometry3d::Identity()};
  double best_turn = 1e9;
  for (const auto & T : graspCandidates(T_box, size)) {
    const double risk = std::round(doublePickRisk(T, neighbors) * 1000.0) / 1000.0;
    const double turn = T_tcp_now ? std::abs(std::remainder(yawOf(T) - yawOf(*T_tcp_now), 2 * M_PI)) : 0.0;
    if (risk < best.first || (risk == best.first && turn < best_turn)) { best = {risk, T}; best_turn = turn; }
  }
  return best;
}

}  // namespace cell_planning
