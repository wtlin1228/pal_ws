#include <gtest/gtest.h>
#include <cell_planning/grasps.hpp>

using namespace cell_planning;
static const BoxSize A{0.30, 0.18, 0.15}, B{0.20, 0.20, 0.12};

TEST(Grasps, LoneBoxHasNoRisk) {
  const auto [risk, T] = bestGrasp(trans(0, 0, 0.075), A, {});
  EXPECT_EQ(risk, 0.0);
  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d(0, 0, 0.15)));
}

TEST(Grasps, SmallBoxBetweenNeighborsIsRisky) {
  const std::vector<Neighbor> neighbors{{trans(0.21, 0, 0.06), B}, {trans(-0.21, 0, 0.06), B}};   // 1 cm gaps left and right
  const auto [risk, T] = bestGrasp(trans(0, 0, 0.06), B, neighbors);
  EXPECT_GT(risk, 0.0);                       // the 0.25 pad overhangs a 0.20 box by 2.5 cm each side
}

TEST(Grasps, CandidatesPointDown) {
  for (const auto & T : graspCandidates(trans(0, 0, 0.075), A))
    EXPECT_TRUE(T.linear().col(2).isApprox(Eigen::Vector3d(0, 0, -1)));
}
