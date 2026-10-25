#include <gtest/gtest.h>

#include <limits>

#include "pid_maze_solver/terminal_clearance.hpp"

namespace
{
using Points = std::vector<std::array<double, 2>>;

Points wall(double y)
{
  Points result;
  for (int i = -30; i <= 30; ++i) result.push_back({i * .01, y});
  return result;
}
}  // namespace

TEST(TerminalClearance, WideStationaryTurnNeedsNoTranslation)
{
  const Points points{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  const auto placement = maze::terminal_placement(points, M_PI);
  ASSERT_TRUE(placement);
  EXPECT_DOUBLE_EQ((*placement)[0], 0);
  EXPECT_DOUBLE_EQ((*placement)[1], 0);
  EXPECT_GT(maze::terminal_sweep_clearance(points, 0, 0, M_PI), .7);
}

TEST(TerminalClearance, MovesAwayFromSingleWallWithShortestGridStep)
{
  const auto points = wall(.23);
  EXPECT_LT(maze::terminal_sweep_clearance(points, 0, 0, M_PI), .025);
  const auto placement = maze::terminal_placement(points, M_PI);
  ASSERT_TRUE(placement);
  EXPECT_LT((*placement)[1], 0);
  EXPECT_NEAR(std::hypot((*placement)[0], (*placement)[1]), .015, 1e-12);
  EXPECT_GE(maze::terminal_sweep_clearance(points, (*placement)[0], (*placement)[1], M_PI), .025);
  EXPECT_TRUE(maze::terminal_detail::translation_clear(points, (*placement)[0], (*placement)[1]));
}

TEST(TerminalClearance, NarrowOppositeWallsHaveNoSolution)
{
  auto points = wall(.23);
  const auto opposite = wall(-.23);
  points.insert(points.end(), opposite.begin(), opposite.end());
  EXPECT_FALSE(maze::terminal_placement(points, M_PI));
  EXPECT_FALSE(maze::terminal_placement(points, -M_PI));
}

TEST(TerminalClearance, TranslationChecksIntermediateBodyClearance)
{
  // Both endpoint poses retain 15 mm, but their straight connector cuts a body corner.
  const Points points{{.185, .136}};
  EXPECT_GE(maze::terminal_sweep_clearance(points, 0, 0, 0), .015);
  EXPECT_GE(maze::terminal_sweep_clearance(points, .014, -.014, 0), .015);
  EXPECT_FALSE(maze::terminal_detail::translation_clear(points, .014, -.014));
  EXPECT_LT(maze::terminal_sweep_clearance(points, .007, -.007, 0), .015);
}

TEST(TerminalClearance, ExistingHardViolationDoesNotAuthorizeEscapeThroughIt)
{
  const Points points{{.184, 0}, {0, .23}};
  EXPECT_FALSE(maze::terminal_placement(points, M_PI));
}

TEST(TerminalClearance, SearchCannotExceedTwentyMillimeterDisk)
{
  // Even a full 20 mm retreat cannot create the 25 mm complete-turn reserve.
  EXPECT_FALSE(maze::terminal_placement(wall(.220), M_PI));
  for (double gap : {.225, .230, .235, .240}) {
    const auto placement = maze::terminal_placement(wall(gap), M_PI);
    if (!placement) continue;
    EXPECT_LE(std::hypot((*placement)[0], (*placement)[1]), .020 + 1e-12);
    EXPECT_NEAR((*placement)[0] / .005, std::round((*placement)[0] / .005), 1e-12);
    EXPECT_NEAR((*placement)[1] / .005, std::round((*placement)[1] / .005), 1e-12);
  }
}

TEST(TerminalClearance, SignedDirectionAndExactEndpointArePreserved)
{
  const Points points{{.21, .05}};
  const double angle = M_PI / 9;
  EXPECT_GT(
    maze::terminal_sweep_clearance(points, 0, 0, angle),
    maze::terminal_sweep_clearance(points, 0, 0, -angle));
  const double endpoint = maze::terminal_detail::pose_clearance(points, 0, 0, -angle);
  EXPECT_LE(maze::terminal_sweep_clearance(points, 0, 0, -angle), endpoint);
  EXPECT_NEAR(
    maze::terminal_sweep_clearance(points, 0, 0, M_PI),
    maze::terminal_sweep_clearance(points, 0, 0, -M_PI), 1e-12);
}

TEST(TerminalClearance, EmptyNonfiniteAndUnsupportedAnglesAreRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(maze::terminal_placement({}, M_PI), std::invalid_argument);
  EXPECT_THROW(maze::terminal_sweep_clearance({}, 0, 0, M_PI), std::invalid_argument);
  for (const Points & bad : {Points{{nan, 1}}, Points{{1, infinity}}, Points{{1, 1}, {nan, 0}}}) {
    EXPECT_THROW(maze::terminal_placement(bad, M_PI), std::invalid_argument);
    EXPECT_THROW(maze::terminal_sweep_clearance(bad, 0, 0, M_PI), std::invalid_argument);
  }
  const Points good{{1, 1}};
  EXPECT_THROW(maze::terminal_sweep_clearance(good, nan, 0, M_PI), std::invalid_argument);
  EXPECT_THROW(maze::terminal_sweep_clearance(good, 0, infinity, M_PI), std::invalid_argument);
  EXPECT_THROW(maze::terminal_placement(good, nan), std::invalid_argument);
  EXPECT_THROW(maze::terminal_placement(good, 2 * M_PI), std::invalid_argument);
}
