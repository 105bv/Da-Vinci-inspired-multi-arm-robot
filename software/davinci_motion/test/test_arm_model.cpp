#include <gtest/gtest.h>
#include <limits>
#include "davinci_motion/arm_model.hpp"

using davinci_motion::ArmModel;
using davinci_motion::Point;
using davinci_motion::pi;
using davinci_motion::radians;

class Models : public ::testing::TestWithParam<int> {};

TEST_P(Models, ReachableGridRoundTrips)
{
  ArmModel model(GetParam());
  int reachable = 0;
  for (double x : {0.18, 0.24, 0.30, 0.36}) {
    for (double y : {-0.16, 0.0, 0.16}) {
      for (double z : {0.04, 0.12, 0.22}) {
        Point p{x, y, z};
        davinci_motion::Joints q;
        try {q = model.inverse(p);} catch (const std::invalid_argument &) {continue;}
        ++reachable;
        auto pose = model.forward(q);
        EXPECT_LT(ArmModel::distance(pose.tip(), p), 1e-9);
        if (GetParam() == 4) {EXPECT_NEAR(pose.pitch, -pi / 2, 1e-9);}
      }
    }
  }
  EXPECT_GE(reachable, 25);
}

TEST_P(Models, RejectsUnreachableSingularAndNonFiniteInputs)
{
  ArmModel model(GetParam());
  EXPECT_THROW(model.inverse(Point{2, 0, 0.1}), std::invalid_argument);
  EXPECT_THROW(model.inverse(Point{0, 0, 0.2}), std::invalid_argument);
  EXPECT_THROW(model.inverse(Point{0.3, 0, -0.2}), std::invalid_argument);
  EXPECT_THROW(
    model.inverse(Point{std::numeric_limits<double>::quiet_NaN(), 0, 0.1}),
    std::invalid_argument);
  auto q = model.home();
  q[0] = radians(180);
  EXPECT_THROW(model.forward(q), std::invalid_argument);
  EXPECT_THROW(model.forward({0, 0}), std::invalid_argument);
}

TEST_P(Models, AllDefaultWaypointPathsHaveClearance)
{
  ArmModel model(GetParam());
  const auto steps = model.sequence();
  ASSERT_EQ(steps.size(), 8U);
  for (std::size_t i = 1; i < steps.size(); ++i) {
    EXPECT_NO_THROW(model.validate_path(steps[i - 1].positions, steps[i].positions));
  }
  EXPECT_EQ(steps.back().positions, model.home());
}

TEST_P(Models, VerticalTaskDistinguishesDesigns)
{
  ArmModel model(GetParam());
  if (GetParam() == 3) {
    EXPECT_THROW(model.sequence(true), std::invalid_argument);
  } else {
    const auto steps = model.sequence(true);
    for (const auto & step : steps) {
      EXPECT_NEAR(model.forward(step.positions).pitch, -pi / 2, 1e-9);
    }
  }
}

TEST_P(Models, DurationRespectsNominalQuinticLimits)
{
  ArmModel model(GetParam());
  const auto a = model.home();
  auto b = a;
  b[0] = 2.0;
  const double t = ArmModel::duration(a, b);
  EXPECT_LE(1.875 * 2.0 / t, 1.0 + 1e-9);
  EXPECT_LE(5.774 * 2.0 / (t * t), 3.0 + 1e-9);
}

TEST(Orientation, WristChangesPitchWhileKeepingPosition)
{
  ArmModel model(4);
  const Point p{0.30, 0, 0.18};
  const auto a = model.forward(model.inverse(p, radians(-90)));
  const auto b = model.forward(model.inverse(p, radians(-45)));
  EXPECT_LT(ArmModel::distance(a.tip(), b.tip()), 1e-9);
  EXPECT_NEAR(b.pitch - a.pitch, radians(45), 1e-9);
}

INSTANTIATE_TEST_SUITE_P(ThreeAndFourDOF, Models, ::testing::Values(3, 4));
