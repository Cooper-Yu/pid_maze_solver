/** @file
 * @brief Reusable scalar PID calculations for planar and angular control.
 */
#ifndef PID_MAZE_SOLVER__PID_HPP_
#define PID_MAZE_SOLVER__PID_HPP_
#include <algorithm>
#include <cmath>

namespace maze
{
/** @brief Scalar PID with conditional integration and command slew limiting. */
class AxisPid
{
public:
  /** @brief Configure gains and bounds from PIDMazeSolver constructor parameters.
   *
 * @param[in] kp Proportional gain, read for position or heading error feedback.
   *
 * @param[in] ki Integral gain, read for accumulated error feedback.
   *
 * @param[in] kd Derivative gain, read for measured velocity damping.
   *
 * @param[in] speed Maximum absolute output velocity, m/s or rad/s.
   *
 * @param[in] acceleration Maximum output slew, m/s squared or rad/s squared.
   */
  AxisPid(double kp, double ki, double kd, double speed, double acceleration)
  : kp_{kp}, ki_{ki}, kd_{kd}, speed_{speed}, acceleration_{acceleration}
  {
  }

  /** @brief Clear accumulated error and previous output after stopping or changing targets. */
  void reset()
  {
    integral_ = command_ = 0.0;
  }

  /** @brief Compute axis velocity for one valid control interval.
   *
 * @param[in] error Target minus measured position (m) from PIDMazeSolver::move(), or wrapped yaw error (rad) from PIDMazeSolver::tick().
   *
 * @param[in] rate Measured odom axis velocity (m/s) or angular.z (rad/s), read for damping.
   *
 * @param[in] dt Positive ROS-time interval supplied by PIDMazeSolver::tick(), seconds.
   *
 * @param[in] speed_limit move() supplies a per-tick translation bound (m/s); zero from
 * tick() keeps the configured angular limit. Reads only; cannot raise the configured cap.
 * @return Bounded m/s or rad/s command returned to PIDMazeSolver::tick() or move() for body-frame conversion, correction and guarded publication; zero for invalid input.
   *
 * @note Writes internal integral/output; integration is blocked when pushing saturation. Lower caps apply immediately;
 * measured-velocity collision prediction remains the caller's responsibility.
   */
  double update(double error, double rate, double dt, double speed_limit = 0.0)
  {
    if (
      !std::isfinite(error) || !std::isfinite(rate) || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(speed_limit) || speed_limit < 0.0) {
      reset();
      return 0.0;
    }
    const double bound = speed_limit > 0 ? std::min(speed_, speed_limit) : speed_;
    const double candidate = std::clamp(integral_ + error * dt, -0.5, 0.5);
    const double requested = kp_ * error + ki_ * candidate - kd_ * rate;
    if (std::abs(requested) <= bound || error * requested < 0.0) integral_ = candidate;
    const double limited = std::clamp(kp_ * error + ki_ * integral_ - kd_ * rate, -bound, bound);
    command_ += std::clamp(limited - command_, -acceleration_ * dt, acceleration_ * dt);
    command_ = std::clamp(command_, -bound, bound);
    return command_;
  }

private:
  double kp_{};            ///< Proportional gain, inverse seconds.
  double ki_{};            ///< Integral gain, inverse seconds squared.
  double kd_{};            ///< Dimensionless measured-rate damping gain.
  double speed_{};         ///< Absolute axis velocity bound, m/s or rad/s.
  double acceleration_{};  ///< Axis velocity slew bound, m/s squared or rad/s squared.
  double integral_{};      ///< Error integral, m-seconds or rad-seconds, bounded to +/-0.5.
  double command_{};       ///< Previous emitted axis velocity, m/s or rad/s.
};
}  // namespace maze
#endif
