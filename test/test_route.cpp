#include <gtest/gtest.h>

#include <limits>

#include "pid_maze_solver/motion.hpp"
#include "pid_maze_solver/pid.hpp"
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

TEST(Motion, P14ClearanceChangePreservesFinalDestinationAndHeadings)
{
  auto reference = maze::default_steps;
  reference[12].forward = .448;
  reference[13].forward = .564;
  reference[13].left = 0;
  const auto before = maze::generate_route(reference);
  const auto after = maze::generate_route(maze::default_steps);
  EXPECT_NEAR(after[13].x - before[13].x, .07 / std::sqrt(2.0), 1e-10);
  EXPECT_NEAR(after[13].y - before[13].y, -.07 / std::sqrt(2.0), 1e-10);
  EXPECT_NEAR(after.back().x, before.back().x, 1e-10);
  EXPECT_NEAR(after.back().y, before.back().y, 1e-10);
  for (std::size_t i = 0; i < before.size(); ++i) EXPECT_DOUBLE_EQ(after[i].yaw, before[i].yaw);
}

TEST(Motion, P08AndP10ApproachesPreserveLaterDestinations)
{
  auto reference = maze::default_steps;
  reference[5].left = -.34;
  reference[6].side_centering = true;
  reference[7].left = -.615;
  reference[8].left = .03;
  const auto before = maze::generate_route(reference);
  const auto after = maze::generate_route(maze::default_steps);
  for (std::size_t i : {6U, 7U, 8U}) {
    EXPECT_NEAR(after[i].x - before[i].x, .03, 1e-12);
    EXPECT_NEAR(after[i].y, before[i].y, 1e-12);
  }
  for (std::size_t i = 9; i < before.size(); ++i) {
    EXPECT_NEAR(after[i].x, before[i].x, 1e-12);
    EXPECT_NEAR(after[i].y, before[i].y, 1e-12);
    EXPECT_DOUBLE_EQ(after[i].yaw, before[i].yaw);
  }
}

TEST(Pid, DynamicCapCannotRaiseConfiguredLimitAndDropsImmediately)
{
  maze::AxisPid pid(1.5, 0.0, 0.0, .20, .25);
  EXPECT_DOUBLE_EQ(pid.update(10, 0, 1, .5), .20);
  EXPECT_DOUBLE_EQ(pid.update(10, 0, .02, .12), .12);
  EXPECT_LE(pid.update(10, 0, .02, .20), .12500001);
  EXPECT_DOUBLE_EQ(pid.update(10, 0, .02, -1), 0);
  EXPECT_DOUBLE_EQ(pid.update(10, 0, .02, NAN), 0);
}

TEST(Motion, RejectInvalidCruiseBounds)
{
  auto steps = maze::default_steps;
  for (double value : {0.0, -1.0, static_cast<double>(NAN)}) {
    steps[0].max_speed = value;
    EXPECT_THROW(maze::generate_route(steps), std::invalid_argument);
  }
}

TEST(Guard, CruisePreviewSeparatesParallelWallFromClosingCorner)
{
  EXPECT_GT(maze::translation_clearance({{0, .22}, {.2, .22}}, .12, 0), .04);
  EXPECT_LT(maze::translation_clearance({{.30, 0}}, .12, 0), .04);
  EXPECT_LT(maze::translation_clearance({{.195, .166}}, .12, 0), .04);
  EXPECT_GT(maze::translation_clearance({{-.30, 0}}, .12, 0), .04);
  EXPECT_LT(maze::translation_clearance({{0, -.25}}, 0, -.12), .04);
}
