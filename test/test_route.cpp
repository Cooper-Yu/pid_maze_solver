#include <gtest/gtest.h>

#include <limits>

#include "pid_maze_solver/motion.hpp"
#include "pid_maze_solver/route.hpp"

TEST(Route, PreserveStrafingHeadings)
{
  EXPECT_DOUBLE_EQ(maze::route[6].yaw, M_PI / 2);
  EXPECT_DOUBLE_EQ(maze::route[8].yaw, M_PI / 2);
  EXPECT_DOUBLE_EQ(maze::route[11].yaw, M_PI);
}

TEST(Guard, FootprintClearance)
{
  EXPECT_NEAR(maze::clearance(.30, 0), .13, 1e-12);
  EXPECT_LT(maze::clearance(0, 0), 0);
  EXPECT_NEAR(maze::clearance(.20, .20), std::hypot(.03, .065), 1e-12);
}

TEST(Route, FinalAndWrap)
{
  EXPECT_NEAR(maze::wrap(-M_PI * 1.5), M_PI * .5, 1e-12);
  EXPECT_EQ(maze::route.size(), 15u);
  EXPECT_NEAR(maze::route.back().x, .09066257469445216, 1e-12);
}

TEST(Guard, EmptyCornerIsNotChassis)
{
  EXPECT_GT(maze::clearance(-.175, -.174), .03);
  EXPECT_LT(maze::clearance(.13, .15), 0);
}

TEST(Route, CoordinateOverridePreservesAllHeadings)
{
  std::vector<double> xy;
  for (const auto & p : maze::route) {
    xy.push_back(p.x);
    xy.push_back(p.y);
  }
  xy[6] += .03;
  const auto candidate = maze::with_positions(xy);
  EXPECT_NEAR(candidate[3].x, maze::route[3].x + .03, 1e-12);
  for (std::size_t i = 0; i < candidate.size(); ++i)
    EXPECT_DOUBLE_EQ(candidate[i].yaw, maze::route[i].yaw);
  EXPECT_DOUBLE_EQ(maze::with_positions({})[3].x, maze::route[3].x);
}

TEST(Route, RejectInvalidCoordinateOverride)
{
  EXPECT_THROW(maze::with_positions({0, 0}), std::invalid_argument);
  std::vector<double> xy(30, 0.0);
  xy[0] = 1.0;
  EXPECT_THROW(maze::with_positions(xy), std::invalid_argument);
  xy[0] = 0.0;
  xy[6] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(maze::with_positions(xy), std::invalid_argument);
}

TEST(Motion, GenerateTargetsFromBodyActions)
{
  const auto r = maze::generate_route(
    {{"forward", 0, 1, 0}, {"right_turn", -M_PI / 2, 1, 0}, {"strafe", 0, 0, -.5}});
  EXPECT_NEAR(r[1].x, 1, 1e-12);
  EXPECT_NEAR(r[2].y, -1, 1e-12);
  EXPECT_NEAR(r[3].x, .5, 1e-12);
  EXPECT_NEAR(r[3].y, -1, 1e-12);
  EXPECT_NEAR(r[3].yaw, -M_PI / 2, 1e-12);
}

TEST(Motion, RejectInvalidSteps)
{
  EXPECT_THROW(maze::generate_route({}), std::invalid_argument);
  EXPECT_THROW(maze::generate_route({{"bad", 4, 0, 0}}), std::invalid_argument);
  EXPECT_THROW(maze::generate_route({{"bad", 0, NAN, 0}}), std::invalid_argument);
}

TEST(Walls, CenterOnlyWithConsistentTwoSidedSupport)
{
  std::vector<std::array<double, 2>> points;
  for (int i = -10; i <= 10; ++i) {
    points.push_back({i * .009, .28});
    points.push_back({i * .009, -.26});
  }
  const auto left = maze::fit_side(points, 1), right = maze::fit_side(points, -1);
  EXPECT_TRUE(maze::corridor(left, right));
  EXPECT_NEAR((left.distance - right.distance) / 2, .01, 1e-12);
  EXPECT_FALSE(maze::corridor(left, {}));
  EXPECT_FALSE(maze::corridor(left, {true, .6, 0}));
  EXPECT_FALSE(maze::corridor(left, {true, .26, .2}));
}
