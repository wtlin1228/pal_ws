#include <string>
#include <gtest/gtest.h>
#include <cell_planning/mixed.hpp>
#include <cell_planning/skus.hpp>

using namespace cell_planning;

TEST(Mixed, SequenceIsConsistent) {
  MixedPlan plan;
  for (char s : std::string("ABCABCABCABC")) {
    if (!plan.target(SKUS.at(std::string(1, s)))) break;
    plan.confirm();
  }
  EXPECT_GE(plan.placed().size(), 8u);
  for (const auto & [T, sz] : plan.placed()) {                       // every box inside the deck, with its yaw
    const auto [w, d] = footprint(sz, yawOf(T));
    EXPECT_LE(std::abs(T.translation().x()) + w / 2, 0.4 + 1e-6);
    EXPECT_LE(std::abs(T.translation().y()) + d / 2, 0.3 + 1e-6);
  }
  EXPECT_LE(plan.heightMap().heights().maxCoeff(), 0.6);
}

TEST(Mixed, SameLevelNeighborsKeepAGap) {
  MixedPlan plan;
  for (int i = 0; i < 6; ++i) { ASSERT_TRUE(plan.target(SKU_A)); plan.confirm(); }
  const auto & P = plan.placed();
  for (std::size_t i = 0; i < P.size(); ++i)
    for (std::size_t j = i + 1; j < P.size(); ++j) {
      const auto & [Ta, a] = P[i]; const auto & [Tb, b] = P[j];
      if (std::abs(Ta.translation().z() - Tb.translation().z()) > 1e-6) continue;
      const auto [wa, da] = footprint(a, yawOf(Ta)); const auto [wb, db] = footprint(b, yawOf(Tb));
      const double gx = std::abs(Ta.translation().x() - Tb.translation().x()) - (wa + wb) / 2;
      const double gy = std::abs(Ta.translation().y() - Tb.translation().y()) - (da + db) / 2;
      EXPECT_GE(std::max(gx, gy), 0.02 - 1e-6);
    }
}
