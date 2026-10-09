#include <gtest/gtest.h>

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
