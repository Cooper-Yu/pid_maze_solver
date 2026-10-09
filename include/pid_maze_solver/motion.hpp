/** @file
 * @brief Relative motion planning and conservative bilateral wall measurements.
 */
#pragma once
#include <limits>
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
  bool side_centering{
    true};  ///< Allow bilateral centering; disable for a planned turn-clearance offset.
  double max_speed{.24};  ///< Cruise cap in m/s, bounded by global and near-wall/arrival limits.
};

/** @brief Editable distances derived from the measured route, without stored target XY poses.
 * @note P07/P08/P09 shift 30 mm right, compensated at P10, with P08 centering disabled.
 * P13_P14 stops 70 mm earlier for corner clearance. P14_P15 compensates both
 * route axes by 70/sqrt(2) mm to preserve the final nominal destination and headings.
 */
inline const std::vector<MotionStep> default_steps{
  {"P01_P02", 0, .35, 0},
  {"P02_P03", -M_PI / 4, .21, 0, true, .12},
  {"P03_P04", -M_PI / 4, 1.15, .03},
  {"P04_P05", M_PI / 2, .46, -.012},
  {"P05_P06", M_PI / 2, .50, -.03},
  {"P06_P07", 0, 0, -.37, true, .20},
  {"P07_P08", 0, .55, 0, false, .20},
  {"P08_P09", 0, 0, -.615, true, .20},
  {"P09_P10", 0, .796, .06, false},
  {"P10_P11", M_PI / 2, .441, 0},
  {"P11_P12", 0, 0, .256, true, .20},
  {"P12_P13", 0, .528, 0},
  {"P13_P14", -M_PI / 4, .378, 0, true, .12},
  {"P14_P15", M_PI / 4, .6134974746830583, -.0494974746830583},
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
    if (!std::isfinite(step.max_speed) || step.max_speed <= 0)
      throw std::invalid_argument("segment speed must be finite and positive");
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

/** @brief Estimate footprint clearance along a short planned translation for cruise slowdown.
 * @param[in] points TF-transformed base_link scan points from PIDMazeSolver::scan().
 * @param[in] dx Body-x preview displacement from translation_limit(), meters.
 * @param[in] dy Body-y preview displacement from translation_limit(), meters.
 * @return Minimum modeled gap (m), read by translation_limit() to reduce cruise speed.
 * @note Samples a straight preview including the current footprint, not rotation or measured
 * momentum. The independent safe() guard still handles those before publication.
 */
inline double translation_clearance(
  const std::vector<std::array<double, 2>> & points, double dx, double dy)
{
  double result = std::numeric_limits<double>::infinity();
  for (const auto & point : points)
    for (int k = 0; k <= 6; ++k)
      result = std::min(result, clearance(point[0] - dx * k / 6, point[1] - dy * k / 6));
  return result;
}

/** @brief Accumulate signed rotation without losing direction at the +/-pi branch cut.
 * @param[in] accumulated Prior terminal progress stored by PIDMazeSolver::odom(), radians.
 * @param[in] previous Last accepted odom yaw stored by odom(), radians.
 * @param[in] current New accepted odom yaw read by odom(), radians.
 * @return Updated progress written back to final_rotation_ and read by tick().
 * @note Caller rejects discontinuous odom before this update; adjacent increments must be <pi.
 */
inline double accumulate_rotation(double accumulated, double previous, double current)
{
  return accumulated + wrap(current - previous);
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