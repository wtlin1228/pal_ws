#include <gtest/gtest.h>
#include <cell_planning/patterns.hpp>

using namespace cell_planning;

TEST(Patterns, LayersAreValid) {
  for (const auto * L : {&layerA(), &layerB()}) { EXPECT_EQ(L->size(), 6u); EXPECT_TRUE(fits(*L)); EXPECT_FALSE(overlaps(*L)); }
}

TEST(Patterns, StackAlternates) {
  const auto s = stack(3);
  EXPECT_EQ(s[0][0].yaw, 0.0); EXPECT_EQ(s[1][0].yaw, M_PI / 2); EXPECT_EQ(s[2][0].yaw, 0.0);   // B starts with the rotated row
}

TEST(Patterns, PlaceOrderFarFirst) {
  const auto order = placeOrder(layerA());
  for (std::size_t i = 1; i < order.size(); ++i) EXPECT_GE(order[i - 1].y, order[i].y);
}

TEST(Patterns, BigSkuDoesNotFitWithGap) {
  const BoxSize C{0.40, 0.30, 0.20};
  EXPECT_FALSE(fits(layerFromRows({{2, 0.0}, {2, 0.0}}, C), C));   // 0.4 + 0.02 + 0.4 > 0.8
}

TEST(Patterns, InterlockBeatsColumnStacking) {                      // Ch 12 checkpoint
  EXPECT_GT(seamOverlap(layerA(), layerB()), seamOverlap(layerA(), layerA()));
}
