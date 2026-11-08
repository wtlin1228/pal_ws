#include <gtest/gtest.h>
#include <cell_planning/geometry.hpp>

using namespace cell_planning;

TEST(Geometry, TopDownTcpPointsDown) {
  const Isometry3d T_box = trans(0.2, 0.1, 0.075) * rotZ(0.5);
  const Isometry3d T_tcp = topDownTcp(T_box, 0.15);
  EXPECT_TRUE(T_tcp.translation().isApprox(Eigen::Vector3d(0.2, 0.1, 0.15)));     // at the top face
  EXPECT_TRUE(T_tcp.linear().col(2).isApprox(Eigen::Vector3d(0, 0, -1)));         // z axis points down
  EXPECT_TRUE(T_tcp.linear().col(0).isApprox(T_box.linear().col(0)));             // x follows the box yaw
}

TEST(Geometry, PoseRoundTrip) {                                                      // Ch 4 checkpoint
  const Isometry3d T = trans(0.3, -0.2, 0.5) * rotX(0.3) * rotZ(1.2);
  EXPECT_TRUE(matFromPose(poseFromMat(T)).isApprox(T, 1e-9));
}
