/** @file
 * @brief Relative motion planning and conservative bilateral wall measurements.
 */
#pragma once
#include <string>

#include "pid_maze_solver/route.hpp"

namespace maze
{
/** @brief A relative turn followed by heading-held body-frame translation. */
struct MotionStep
{
  std::string name;  ///< Parameter namespace and human-readable segment name.
  double turn;       ///< Signed change from previous planned heading, radians.
  double forward;    ///< Signed travel along the new heading, meters.
  double left;       ///< Signed travel left of the new heading, meters; negative is right.
};

/** @brief Editable distances derived from the measured route, without stored target XY poses. */
inline const std::vector<MotionStep> default_steps{
  {"P01_P02", 0, .35, 0},
  {"P02_P03", -M_PI / 4, .21, 0},
  {"P03_P04", -M_PI / 4, 1.15, .03},
  {"P04_P05", M_PI / 2, .46, -.012},
  {"P05_P06", M_PI / 2, .50, -.03},
  {"P06_P07", 0, 0, -.34},
  {"P07_P08", 0, .55, 0},
  {"P08_P09", 0, 0, -.615},
  {"P09_P10", 0, .796, 0},
  {"P10_P11", M_PI / 2, .471, 0},
  {"P11_P12", 0, 0, .256},
  {"P12_P13", 0, .528, 0},
  {"P13_P14", -M_PI / 4, .448, 0},
  {"P14_P15", M_PI / 4, .564, 0},
};

/** @brief Generate nominal poses cumulatively from relative actions at a zero route origin.
 * @par Input to target flow
 * PIDMazeSolver reads named step parameters once, calls this function, then anchors these
 * poses to the first stopped odometry pose. Changing a distance moves subsequent targets.
 * @param[in] steps Constructor-owned actions; reads turn and body-frame displacements.
 * @return Generated P01 plus one target per step, stored in PIDMazeSolver::route_.
 * @note No measured stopping error is accumulated into later nominal targets.
 */
inline std::vector<Point> generate_route(const std::vector<MotionStep> & steps)
{
  if (steps.empty() || steps.size() > 64) throw std::invalid_argument("require 1..64 steps");
  std::vector<Point> result{{0, 0, 0}};
  for (const auto & step : steps) {
    if (!std::isfinite(step.turn) || !std::isfinite(step.forward) || !std::isfinite(step.left))
      throw std::invalid_argument("motion steps must be finite");
    if (std::abs(step.turn) > M_PI)
      throw std::invalid_argument("turn must be within +/-180 degrees");
    const auto & previous = result.back();
    const double yaw = previous.yaw + step.turn, c = std::cos(yaw), s = std::sin(yaw);
    result.push_back(
      {previous.x + c * step.forward - s * step.left, previous.y + s * step.forward + c * step.left,
       yaw});
  }
  return result;
}

/** @brief One side-wall fit; invalid fits must never drive centering. */
struct SideWall
{
  bool valid{};       ///< True only for enough aligned, low-residual side returns.
  double distance{};  ///< Positive lateral distance from base_link to the fitted wall, meters.
  double angle{};     ///< Wall tangent relative to body x, radians in [-pi/2,pi/2].
};

/** @brief Fit a local side wall, rejecting openings, corners and inadequate support.
 * @param[in] points TF-transformed base_link points from PIDMazeSolver::scan(), meters.
 * @param[in] side +1 selects left; -1 selects right.
 * @return Valid fit for bilateral centering, or an invalid value for odom-only tracking.
 * @note Requires 12 returns, 0.12 m span, RMS <=8 mm, angle <=10 degrees and range .18..0.40 m.
 */
inline SideWall fit_side(const std::vector<std::array<double, 2>> & points, int side)
{
  std::vector<std::array<double, 2>> selected;
  double mx = 0, my = 0, minx = 1e9, maxx = -1e9;
  for (const auto & p : points) {
    if (side * p[1] < .18 || side * p[1] > .4 || std::abs(p[0]) > .364 * std::abs(p[1])) continue;
    selected.push_back(p);
    mx += p[0];
    my += p[1];
    minx = std::min(minx, p[0]);
    maxx = std::max(maxx, p[0]);
  }
  if (selected.size() < 12 || maxx - minx < .12) return {};
  mx /= selected.size();
  my /= selected.size();
  double xx = 0, yy = 0, xy = 0;
  for (const auto & p : selected) {
    double x = p[0] - mx, y = p[1] - my;
    xx += x * x;
    yy += y * y;
    xy += x * y;
  }
  const double a = .5 * std::atan2(2 * xy, xx - yy);
  const double residual = std::max(0.0, (xx + yy - std::hypot(xx - yy, 2 * xy)) / 2);
  if (std::abs(a) > M_PI / 18 || std::sqrt(residual / selected.size()) > .008) return {};
  return {true, side * (my - std::tan(a) * mx), a};
}

/** @brief Accept centering only for two consistent walls of a narrow corridor.
 * @param[in] left Wall estimate from fit_side(points,+1).
 * @param[in] right Wall estimate from fit_side(points,-1).
 * @return True for parallel supported walls with total width .45..65 m.
 * @note One missing wall, an opening, or an inconsistent width disables new correction.
 */
inline bool corridor(const SideWall & left, const SideWall & right)
{
  return left.valid && right.valid && std::abs(left.angle - right.angle) < M_PI / 36 &&
         left.distance + right.distance >= .45 && left.distance + right.distance <= .65;
}
}  // namespace maze