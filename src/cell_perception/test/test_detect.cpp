// The detector on synthetic scenes (same recipe as the mock camera).
#include <random>
#include <gtest/gtest.h>
#include <cell_perception/heightmap.hpp>

static void face(std::vector<Eigen::Vector3d> & P, double cx, double cy, double sx, double sy, double z, double step = 0.005) {
  for (double u = -sx / 2; u <= sx / 2 + 1e-9; u += step)
    for (double v = -sy / 2; v <= sy / 2 + 1e-9; v += step) P.emplace_back(cx + u, cy + v, z);
}

static std::vector<Eigen::Vector3d> scene(const std::vector<std::pair<double, double>> & boxes, double sx, double sy, double h) {
  std::vector<Eigen::Vector3d> P;
  face(P, 0, 0, 0.8, 0.6, 0.0);                                   // the deck
  for (auto [x, y] : boxes) face(P, x, y, sx, sy, h);              // the box tops
  std::vector<Eigen::Vector3d> visible;
  std::mt19937 rng(0); std::normal_distribution<double> noise(0.0, 0.002);
  for (const auto & q : P) {                                       // hide deck points under the boxes
    bool hidden = false;
    for (auto [x, y] : boxes) hidden |= std::abs(q.x() - x) < sx / 2 - 1e-6 && std::abs(q.y() - y) < sy / 2 - 1e-6 && q.z() < h - 1e-3;
    if (!hidden) visible.push_back(q + Eigen::Vector3d(noise(rng), noise(rng), noise(rng)));
  }
  return visible;
}

TEST(Detect, SixBoxesWithGaps) {
  std::vector<std::pair<double, double>> boxes;
  for (double x : {-0.16, 0.16}) for (double y : {-0.2, 0.0, 0.2}) boxes.emplace_back(x, y);
  const auto dets = cell_perception::detect(scene(boxes, 0.30, 0.18, 0.15));
  ASSERT_EQ(dets.size(), 6u);
  for (const auto & d : dets) {
    EXPECT_NEAR(d.sx, 0.30, 0.011); EXPECT_NEAR(d.sy, 0.18, 0.011); EXPECT_NEAR(d.z_top, 0.15, 0.01);
    double best = 1.0;
    for (auto [x, y] : boxes) best = std::min(best, std::hypot(d.x - x, d.y - y));
    EXPECT_LT(best, 0.008);                                        // half a cell per axis
  }
}

TEST(Detect, ThreeSmallBoxesOneCentimeterApart) {
  const std::vector<std::pair<double, double>> boxes{{-0.21, 0.0}, {0.0, 0.0}, {0.21, 0.0}};
  const auto dets = cell_perception::detect(scene(boxes, 0.20, 0.20, 0.12));
  EXPECT_EQ(dets.size(), 3u);
}

TEST(Detect, EmptyPallet) {
  EXPECT_TRUE(cell_perception::detect(scene({}, 0.3, 0.18, 0.15)).empty());
}
