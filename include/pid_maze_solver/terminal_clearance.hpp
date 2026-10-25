/** @file
 * @brief Sampled terminal placement against the unchanged body/wheel union model.
 */
#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "pid_maze_solver/route.hpp"

namespace maze
{
namespace terminal_detail
{
using ScanPoints = std::vector<std::array<double, 2>>;

/** @brief Reject incomplete/nonfinite input rather than deleting obstacle points. */
inline void validate(const ScanPoints & points, double dx, double dy, double angle)
{
  if (
    points.empty() || !std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(angle) ||
    std::abs(angle) > M_PI + 1e-12) {
    throw std::invalid_argument("Terminal geometry requires finite points and angle within +/-pi");
  }
  for (const auto & point : points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
      throw std::invalid_argument("Nonfinite terminal obstacle point");
    }
  }
}

/** @brief Minimum model gap at one translated/rotated body pose. */
inline double pose_clearance(const ScanPoints & points, double dx, double dy, double angle)
{
  const double c = std::cos(angle), s = std::sin(angle);
  double gap = std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    const double x = point[0] - dx, y = point[1] - dy;
    gap = std::min(gap, clearance(c * x + s * y, -s * x + c * y));
  }
  return gap;
}

/** @brief Require every sampled translation pose, including start/end, to retain 15 mm. */
inline bool translation_clear(const ScanPoints & points, double dx, double dy)
{
  const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / .0025)));
  for (int i = 0; i <= count; ++i) {
    const double fraction = static_cast<double>(i) / count;
    if (pose_clearance(points, dx * fraction, dy * fraction, 0.0) < .015) return false;
  }
  return true;
}
}  // namespace terminal_detail

/** @brief Minimum signed-turn model gap; <=2-degree sampling includes start and exact finish.
 * @param points All accepted obstacle points in the initial body frame, meters.
 * @param dx Placement translation forward in the initial body frame, meters.
 * @param dy Placement translation left in the initial body frame, meters.
 * @param angle Signed turn from zero, bounded to +/-pi radians.
 * @return Minimum sampled clearance to the unchanged body/wheel union, meters.
 * @throws std::invalid_argument for empty/nonfinite data or unsupported angular range.
 */
inline double terminal_sweep_clearance(
  const std::vector<std::array<double, 2>> & points, double dx, double dy, double angle)
{
  terminal_detail::validate(points, dx, dy, angle);
  const int count = std::max(1, static_cast<int>(std::ceil(std::abs(angle) / (M_PI / 90.0))));
  double gap = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= count; ++i) {
    gap = std::min(
      gap, terminal_detail::pose_clearance(points, dx, dy, angle * static_cast<double>(i) / count));
  }
  return gap;
}

/** @brief Shortest sampled-safe 5 mm lattice placement within a 20 mm Euclidean disk.
 *
 * Retains every obstacle point. The translation retains >=15 mm sampled clearance;
 * the entire subsequent signed turn retains >=25 mm sampled clearance. This is
 * sensor/model geometry only, not proof of physical contact-free execution.
 * @return Initial-body-frame {forward,left} displacement, or nullopt if none qualifies.
 */
inline std::optional<std::array<double, 2>> terminal_placement(
  const std::vector<std::array<double, 2>> & points, double angle)
{
  terminal_detail::validate(points, 0.0, 0.0, angle);
  if (terminal_sweep_clearance(points, 0.0, 0.0, angle) >= .025) return std::array<double, 2>{0, 0};
  std::optional<std::array<double, 2>> best;
  double shortest = std::numeric_limits<double>::infinity();
  double best_gap = -std::numeric_limits<double>::infinity();
  for (int ix = -4; ix <= 4; ++ix) {
    for (int iy = -4; iy <= 4; ++iy) {
      const double dx = ix * .005, dy = iy * .005;
      const double length = std::hypot(dx, dy);
      if (length > .020 + 1e-12 || length > shortest + 1e-12) continue;
      if (!terminal_detail::translation_clear(points, dx, dy)) continue;
      const double gap = terminal_sweep_clearance(points, dx, dy, angle);
      if (gap < .025) continue;
      if (length < shortest - 1e-12 || gap > best_gap) {
        best = std::array<double, 2>{dx, dy};
        shortest = length;
        best_gap = gap;
      }
    }
  }
  return best;
}
}  // namespace maze
