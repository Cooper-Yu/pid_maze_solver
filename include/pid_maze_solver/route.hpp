/** @file
 * @brief User supplied route and body-frame geometry. */
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace maze
{
/** @brief Pose in the route frame anchored at the first accepted stopped odometry pose. */
struct Point
{
  double x;    ///< Route-forward coordinate in meters.
  double y;    ///< Route-left coordinate in meters.
  double yaw;  ///< Heading relative to the route origin, radians; retained during translation.
};

/** @brief Preserved user reference points; no tangent-heading substitution. */
inline const std::array<Point, 15> route{{
  Point{0.0, 0.0, 0},
  Point{0.3518789451107292, 0.0, 0},
  Point{0.4991250695014871, -0.14724612439075788, -0.78539816339744828},
  Point{0.49912506950148716, -1.311200258478562, -1.5707963267948966},
  Point{0.9870160350483727, -1.311200258478562, 0},
  Point{0.9870160350483727, -0.7823810385420719, 1.5707963267948966},
  Point{1.3550234073853702, -0.7823810385420719, 1.5707963267948966},
  Point{1.3550234073853702, -0.2583804040891572, 1.5707963267948966},
  Point{1.97, -0.2583804040891572, 1.5707963267948966},
  Point{1.97, 0.5376209143926302, 1.5707963267948966},
  Point{1.499105068828838, 0.5376209143926302, 3.1415926535897931},
  Point{1.499105068828838, 0.2816671367685469, 3.1415926535897931},
  Point{0.9711047467614911, 0.28166713676854693, 3.1415926535897931},
  Point{0.6546225628284905, 0.5981493207015476, 2.3561944901923448},
  Point{0.09066257469445216, 0.5981493207015477, 3.1415926535897931},
}};

/** @brief Apply optional measured XY coordinates without changing any supplied heading.
 * @par Coordinate calibration
 * An empty vector selects the original route. A complete override contains all 15 XY pairs.
 * @param[in] xy Parameter vector read by PIDMazeSolver's constructor; route-frame meters.
 * @return Route copied into PIDMazeSolver::route_; the original array and yaw values remain unchanged.
 * @note Rejects incomplete/nonfinite coordinates and a moved P01 origin before motion starts.
 */
inline std::array<Point, 15> with_positions(const std::vector<double> & xy)
{
  auto result = route;
  if (xy.empty()) return result;
  if (xy.size() != 2 * result.size())
    throw std::invalid_argument("waypoint_xy requires all 15 XY pairs");
  for (double v : xy)
    if (!std::isfinite(v)) throw std::invalid_argument("waypoint_xy must be finite");
  if (xy[0] != 0 || xy[1] != 0)
    throw std::invalid_argument("P01 must remain the route origin (0,0)");
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i].x = xy[2 * i];
    result[i].y = xy[2 * i + 1];
  }
  return result;
}

/** @brief Shortest signed angle.
 * @param[in] angle Difference from current to target in radians.
 *
 * @return Equivalent difference in [-pi,pi], used by the control state machine. */
inline double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/** @brief Clearance to the union of model-derived body and wheel bounding rectangles.
 *
 * @param[in] x Laser point x in base_link meters, read by scan protection.
 *
 * @param[in] y Laser point y in base_link meters, read by scan protection.
 *
 * @return Distance to body (+/- .170,.135) or wheel envelope (+/- .135,.160), meters.
 */
inline double clearance(double x, double y)
{
  const auto box = [x, y](double hx, double hy) {
    const double dx = std::abs(x) - hx, dy = std::abs(y) - hy;
    return (dx > 0 || dy > 0) ? std::hypot(std::max(dx, 0.0), std::max(dy, 0.0)) : std::max(dx, dy);
  };
  return std::min(box(.170, .135), box(.135, .160));
}
}  // namespace maze
