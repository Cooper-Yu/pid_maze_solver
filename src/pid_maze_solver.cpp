/** @file
 * @brief One-owner turn/move maze state machine with odom feedback and laser guards. */
#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_maze_solver/motion.hpp"
#include "pid_maze_solver/pid.hpp"
#include "pid_maze_solver/route.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
/** @brief Monotonic clock used for feedback and execution watchdogs. */
using Clock = std::chrono::steady_clock;

/** @brief Execute TURN then MOVE at each supplied pose, preserving destination yaw during holonomic motion. */
class PIDMazeSolver : public rclcpp::Node
{
public:
  /** @brief Configure simulator interfaces, gains and guarded execution without starting motion immediately. */
  PIDMazeSolver() : Node("pid_maze_solver"), buffer_(get_clock()), listener_(buffer_)
  {
    if (!get_node_parameters_interface()->get_parameter_overrides().count("use_sim_time"))
      set_parameter(rclcpp::Parameter("use_sim_time", true));
    configure_route();
    final_turn_enabled_ = declare_parameter<bool>("final_clockwise_turn", true);
    speed_ = positive("max_speed", steps_.empty() ? .12 : .24);
    accel_ = positive("max_acceleration", steps_.empty() ? .25 : .4);
    const double kp = positive("distance_kp", 1.5), ki = nonnegative("distance_ki", 0.0),
                 kd = nonnegative("distance_kd", 0.0);
    px_ = std::make_unique<maze::AxisPid>(kp, ki, kd, speed_, accel_);
    py_ = std::make_unique<maze::AxisPid>(kp, ki, kd, speed_, accel_);
    yaw_pid_ = std::make_unique<maze::AxisPid>(
      positive("turn_kp", 1.8), nonnegative("turn_ki", 0.03), nonnegative("turn_kd", 0.35),
      positive("max_yaw_rate", 0.6), 0.6);
    stage_timeout_ = positive("stage_timeout", 60.0);
    stop_hold_ = positive("stop_hold", steps_.empty() ? .4 : .20);
    last_index_ = declare_parameter<int>("last_point", static_cast<int>(route_.size()));
    if (last_index_ < 2 || last_index_ > static_cast<int>(route_.size()))
      throw std::invalid_argument("last_point must be within generated route, at least 2");
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) { odom(*m); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter<std::string>("scan_topic", "/scan"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) { scan(*m); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    RCLCPP_INFO(
      get_logger(), "Task5: %d points; destination yaw preserved; distance PID=(%.2f,%.2f,%.2f)",
      last_index_, kp, ki, kd);
  }

  /** @brief Return process status to main(); zero only after normal route completion. */
  int result() const
  {
    return code_;
  }

  /** @brief Publish zero at process cleanup; no new motion is requested. */
  void stop()
  {
    pub_->publish(geometry_msgs::msg::Twist{});
  }

private:
  /** @brief Load either named motion steps or the compatibility fixed-point route.
   * @par Route generation
   * Reads node parameters before subscriptions start; relative steps generate every target XY.
   * @note Invalid modes and conflicting XY/step sources throw before motion is possible.
   */
  void configure_route()
  {
    const auto mode = declare_parameter<std::string>("route_mode", "motion_steps");
    const auto xy = declare_parameter<std::vector<double>>("waypoint_xy", std::vector<double>{});
    side_centering_ = declare_parameter<bool>("side_centering", true);
    if (mode == "fixed_points") {
      const auto fixed = maze::with_positions(xy);
      route_.assign(fixed.begin(), fixed.end());
      return;
    }
    if (mode != "motion_steps" || !xy.empty())
      throw std::invalid_argument(
        "motion_steps rejects waypoint_xy; select fixed_points explicitly");
    steps_ = maze::default_steps;
    for (auto & step : steps_) {
      const auto prefix = "steps." + step.name + ".";
      step.turn =
        declare_parameter<double>(prefix + "turn_deg", step.turn * 180 / M_PI) * M_PI / 180;
      step.forward = declare_parameter<double>(prefix + "forward_m", step.forward);
      step.left = declare_parameter<double>(prefix + "left_m", step.left);
      step.max_speed = positive(prefix + "max_speed", step.max_speed);
      step.side_centering = declare_parameter<bool>(prefix + "side_centering", step.side_centering);
    }
    route_ = maze::generate_route(steps_);
    RCLCPP_INFO(
      get_logger(), "Generated %zu targets from relative motion distances", steps_.size());
  }

  /** @brief Read finite nonnegative gain/bound from node parameter overrides.
  *
 * @param[in] key Constructor parameter name.
 * @param[in] value Default read by declaration.
  *
 * @return Validated value stored by the constructor; throws for invalid input. */
  double nonnegative(const std::string & key, double value)
  {
    double v = declare_parameter<double>(key, value);
    if (!std::isfinite(v) || v < 0) throw std::invalid_argument(key);
    return v;
  }

  /** @brief Read a strictly positive bound.
 * @param[in] key Constructor key.
  *
 * @param[in] value Default value.
 * @return Validated bound for controller configuration. */
  double positive(const std::string & key, double value)
  {
    double v = nonnegative(key, value);
    if (v == 0) throw std::invalid_argument(key);
    return v;
  }

  /** @brief Stop and terminate a failed task without advancing a waypoint.
  *
 * @param[in] reason Diagnostic supplied by the guard that failed. */
  void fail(const char * reason)
  {
    stop();
    code_ = 2;
    RCLCPP_ERROR(
      get_logger(), "%s point=P%02zu stage=%s", reason, index_ + 1,
      final_turn_active_ ? "FINAL_TURN" : (moving_ ? "MOVE" : "TURN"));
    rclcpp::shutdown();
  }

  /** @brief Validate feedback and store only advancing, fresh odometry.
  *
 * @param[in] m Subscription message; pose and body velocity copied to persistent state. */
  void odom(const nav_msgs::msg::Odometry & m)
  {
    const rclcpp::Time stamp(m.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || age < -.1 || age > .5 || (have_odom_ && stamp <= odom_stamp_))
      return;
    const auto & q = m.pose.pose.orientation;
    double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    double roll, pitch, yaw;
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > .01) return;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    const auto & p = m.pose.pose.position;
    const auto & v = m.twist.twist;
    if (
      !std::isfinite(norm) || std::abs(norm - 1) > .01 || !std::isfinite(p.x) ||
      !std::isfinite(p.y) || !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) ||
      !std::isfinite(v.angular.z) || m.header.frame_id.empty() || m.child_frame_id != "base_link")
      return;
    if (
      have_odom_ && (m.header.frame_id != frame_ || std::hypot(p.x - x_, p.y - y_) > .2 ||
                     std::abs(maze::wrap(yaw - yaw_)) > .35)) {
      fail("ODOM_DISCONTINUITY");
      return;
    }
    frame_ = m.header.frame_id;
    x_ = p.x;
    y_ = p.y;
    if (final_turn_active_) final_rotation_ = maze::accumulate_rotation(final_rotation_, yaw_, yaw);
    yaw_ = yaw;
    vx_ = v.linear.x;
    vy_ = v.linear.y;
    wz_ = v.angular.z;
    odom_stamp_ = stamp;
    odom_received_ = Clock::now();
    have_odom_ = true;
  }

  /** @brief Transform finite laser returns into base_link using the scan frame's TF.
  *
 * @param[in] m Scan callback message; valid points replace the collision/correction snapshot.
  *
 * @note Invalid/stale scans do not refresh the watchdog. Infinite returns are not wall points. */
  void scan(const sensor_msgs::msg::LaserScan & m)
  {
    const rclcpp::Time stamp(m.header.stamp, get_clock()->get_clock_type());
    double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || age < -.1 || age > .5 || (have_scan_ && stamp <= scan_stamp_))
      return;
    try {
      auto tf = buffer_.lookupTransform("base_link", m.header.frame_id, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
      std::vector<std::array<double, 2>> points;
      for (std::size_t i = 0; i < m.ranges.size(); ++i) {
        double d = m.ranges[i];
        if (!std::isfinite(d) || d < m.range_min || d > m.range_max) continue;
        double a = m.angle_min + i * m.angle_increment;
        auto p = rotation * tf2::Vector3(d * std::cos(a), d * std::sin(a), 0);
        const double bx = p.x() + tf.transform.translation.x,
                     by = p.y() + tf.transform.translation.y;
        /** Exclude the known base_link chassis self-return box. */
        if (bx >= -.195 && bx <= .165 && std::abs(by) <= .145) continue;
        points.push_back({bx, by});
      }
      if (points.size() < 30) return;
      points_ = std::move(points);
      scan_received_ = Clock::now();
      scan_stamp_ = stamp;
      have_scan_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  /** @brief Reset both PID histories whenever the state changes, preserving fixed route targets. */
  void reset()
  {
    px_->reset();
    py_->reset();
    yaw_pid_->reset();
    holding_ = false;
    blocked_ = false;
  }

  /** @brief Select a waypoint and reuse an already verified stop when no turn is needed.
   * @param[in] qualified_stop complete_stage() supplies true only after its continuous stopped
   * hold. initialize() uses false, preserving startup qualification and the first TURN.
   * @note Reads planned/current yaw and fresh velocities; writes moving_ for the next tick.
   * Real turns, every destination's stopped hold and final stop remain mandatory.
   */
  void target(bool qualified_stop = false)
  {
    const auto & p = route_[index_];
    tx_ = ox_ + std::cos(oyaw_) * p.x - std::sin(oyaw_) * p.y;
    ty_ = oy_ + std::sin(oyaw_) * p.x + std::cos(oyaw_) * p.y;
    heading_ = oyaw_ + p.yaw;
    nominal_tx_ = tx_;
    nominal_ty_ = ty_;
    center_offset_ = 0;
    center_count_ = 0;
    center_scan_ = scan_stamp_;
    turn_x_ = x_;
    turn_y_ = y_;
    moving_ = qualified_stop && std::abs(maze::wrap(p.yaw - route_[index_ - 1].yaw)) < 1e-9 &&
              std::abs(maze::wrap(heading_ - yaw_)) < .01 && std::hypot(vx_, vy_) <= .01 &&
              std::abs(wz_) <= .02 && elapsed(odom_received_) <= .5 &&
              elapsed(scan_received_) <= .5;
    reset();
    stage_started_ = Clock::now();
    stage_ros_started_ = now();
    RCLCPP_INFO(
      get_logger(), "Target P%02zu: x=%.6f y=%.6f yaw=%.6f; %s", index_ + 1, tx_, ty_, heading_,
      moving_ ? "MOVE with reused stop" : "TURN then MOVE");
    if (moving_) {
      RCLCPP_INFO(
        get_logger(), "P%02zu MOVE (reused qualified stop; unchanged heading)", index_ + 1);
    }
  }

  /** @brief Require fresh sensors and stopped feedback before anchoring the entire route.
   *
 * @return False; tick() ends this callback even when target() selects the first destination.
   *
 * @note Records the stopped odom origin; does not physically reposition the robot. */
  bool initialize()
  {
    if (elapsed(started_) > 15) {
      fail("INITIAL_FEEDBACK_TIMEOUT");
      return false;
    }
    if (
      !have_odom_ || !have_scan_ || elapsed(odom_received_) > .5 || elapsed(scan_received_) > .5) {
      stop();
      if (elapsed(started_) > 15) fail("INITIAL_FEEDBACK_TIMEOUT");
      return false;
    }
    if (std::hypot(vx_, vy_) > .01 || std::abs(wz_) > .02) {
      holding_ = false;
      stop();
      return false;
    }
    if (!holding_) {
      holding_ = true;
      hold_ = now();
    }
    stop();
    if ((now() - hold_).seconds() < .4) return false;
    ox_ = x_;
    oy_ = y_;
    oyaw_ = yaw_;
    initialized_ = true;
    last_ = now();
    index_ = 1;
    RCLCPP_INFO(get_logger(), "Route origin P01: x=%.6f y=%.6f yaw=%.6f", ox_, oy_, oyaw_);
    target();
    return false;
  }

  /** @brief Measure wall-clock age for watchdogs.
  *
 * @param[in] t Stored receipt/start time.
 * @return Elapsed steady seconds, independent of ROS pauses. */
  double elapsed(Clock::time_point t) const
  {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }

  /** @brief Hold stopped conditions continuously for the configured ROS-time duration.
  *
 * @param[in] in_tolerance Pose predicate from tick().
 * @return True only after stopped hold completes. */
  bool settled(bool in_tolerance)
  {
    if (!in_tolerance || std::hypot(vx_, vy_) > .01 || std::abs(wz_) > .02) {
      holding_ = false;
      return false;
    }
    if (!holding_) {
      holding_ = true;
      hold_ = now();
    }
    return (now() - hold_).seconds() >= stop_hold_;
  }

  /** @brief Explain turn convergence and stopped-hold gates without modifying control state.
   * @param[in] e tick() or complete_stage() supplies the current signed heading error in radians.
   * @par Diagnostic flow
   * Reads accepted odom velocities, holding_/hold_ after the current tick's gate update,
   * and stage start times. Writes only an INFO log at most every 0.5 steady seconds.
   * @note Angle, angular-speed and linear-speed flags expose all failing gates. The reason
   * selects the first failure; hold starts only after all gates pass. ROS/wall elapsed times
   * help distinguish slow simulation from slow convergence. No commands or PID state change.
   */
  void log_turn_settling(double e)
  {
    if (moving_) return;
    const bool angle_ok = std::abs(e) < .01;
    const bool angular_ok = std::abs(wz_) <= .02;
    const double linear_speed = std::hypot(vx_, vy_);
    const bool linear_ok = linear_speed <= .01;
    const double held = holding_ ? std::max(0.0, (now() - hold_).seconds()) : 0.0;
    const char * reason = !angle_ok           ? "angle"
                          : !angular_ok       ? "angular_speed"
                          : !linear_ok        ? "linear_speed"
                          : held < stop_hold_ ? "hold"
                                              : "ready";
    RCLCPP_INFO_THROTTLE(
      get_logger(), diagnostic_clock_, 500,
      "TURN_SETTLING P%02zu stage=%s reason=%s | error=%.6f rad tol=0.010000 angle_ok=%d | "
      "measured_wz=%.6f limit=0.020000 angular_ok=%d | linear_speed=%.6f limit=0.010000 "
      "linear_ok=%d | hold_ros=%.3f/%.3f s | stage_ros=%.3f stage_wall=%.3f s",
      index_ + 1, final_turn_active_ ? "FINAL_TURN" : "TURN", reason, e, angle_ok, wz_, angular_ok,
      linear_speed, linear_ok, held, stop_hold_, (now() - stage_ros_started_).seconds(),
      elapsed(stage_started_));
  }

  /** @brief Apply laser-based near-wall course correction to a moving body-frame velocity.
  *
 * @param[in,out] cmd Desired body velocity from move(); adds a bounded repulsive component,
  * then returns the corrected command to tick() for swept-footprint protection and publication.
  *
 * @note Correction cannot declare arrival; odom position and heading remain authoritative. */
  void correct(geometry_msgs::msg::Twist & cmd)
  {
    double rx = 0, ry = 0;
    for (const auto & p : points_) {
      double c = maze::clearance(p[0], p[1]);
      if (c >= .07) continue;
      double d = std::hypot(p[0], p[1]);
      if (d < .01) continue;
      double gain = std::clamp((.07 - c) * .5, 0.0, .025);
      double ax = -gain * p[0] / d, ay = -gain * p[1] / d;
      if (std::abs(ax) > std::abs(rx)) rx = ax;
      if (std::abs(ay) > std::abs(ry)) ry = ay;
    }
    const double blend = std::clamp((std::hypot(tx_ - x_, ty_ - y_) - .015) / .10, 0.0, 1.0);
    cmd.linear.x += blend * rx;
    cmd.linear.y += blend * ry;
  }

  /** @brief Predict a conservative swept chassis over a braking horizon from current laser points.
  *
 * @param[in] cmd Candidate body command from tick(), read without alteration.
  *
 * @param[in] escape True for non-worsening clearance recovery; false for normal motion.
  *
 * @return False when current or predicted footprint clearance is below 15 mm; caller stops.
  *
 * @note Includes measured velocity to cover residual movement; no global collision-free claim. */
  bool safe(const geometry_msgs::msg::Twist & cmd, bool escape = false)
  {
    for (const auto & p : points_)
      for (int k = 0; k <= 6; ++k) {
        double t = k * .08;
        for (int v = 0; v < 2; ++v) {
          double x = p[0] - (v ? vx_ : cmd.linear.x) * t, y = p[1] - (v ? vy_ : cmd.linear.y) * t;
          double a = -(v ? wz_ : cmd.angular.z) * t;
          const double before = maze::clearance(p[0], p[1]);
          const double after =
            maze::clearance(std::cos(a) * x - std::sin(a) * y, std::sin(a) * x + std::cos(a) * y);
          const bool unsafe =
            escape ? (before < .005 || after < std::min(.015, before) - .0001) : after < .015;
          if (unsafe) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Blocked point=(%.3f,%.3f) horizon=%.2f measured=%d", p[0], p[1], t, v);
            return false;
          }
        }
      }
    return true;
  }

  /** @brief Adjust the current generated target toward a verified corridor centerline.
   * @par Bounded wall correction
   * Only forward-dominant motion uses bilateral centering. Three fresh consistent fits are
   * required; openings disable further correction and retain the accepted track offset.
   * @note Reads scan points and current odom; writes tx_/ty_ within 6 cm of the nominal target.
   * Later nominal targets remain generated from the original motion sequence, not odom error.
   */
  void center_from_sides()
  {
    if (!side_centering_ || steps_.empty() || !moving_ || scan_stamp_ <= center_scan_) return;
    center_scan_ = scan_stamp_;
    const auto & step = steps_[index_ - 1];
    if (
      !step.side_centering || step.forward < .1 || std::abs(step.left) > .08 ||
      std::abs(maze::wrap(heading_ - yaw_)) > .03)
      return;
    const auto left = maze::fit_side(points_, 1), right = maze::fit_side(points_, -1);
    if (!maze::corridor(left, right)) {
      center_count_ = 0;
      return;
    }
    if (++center_count_ < 3) return;
    const double nx = -std::sin(heading_), ny = std::cos(heading_);
    const double requested = std::clamp(
      (x_ - nominal_tx_) * nx + (y_ - nominal_ty_) * ny + (left.distance - right.distance) / 2,
      -.06, .06);
    center_offset_ += std::clamp(requested - center_offset_, -.004, .004);
    tx_ = nominal_tx_ + nx * center_offset_;
    ty_ = nominal_ty_ + ny * center_offset_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "SIDE_CENTER P%02zu left=%.3f right=%.3f target_offset=%.3f m", index_ + 1, left.distance,
      right.distance, center_offset_);
  }

  /** @brief Bound cruise by segment policy, arrival distance and observed footprint clearance.
   * @return Speed bound consumed by move() for axis PID and the corrected body command.
   * @note Reads current odom/target and scan points. When a 0.12 m straight preview has less than
   * 40 mm footprint clearance, caps at 0.12 m/s. The arrival cap rises smoothly from
   * 0.12 m/s at 0.12 m remaining.
   * This is anticipatory slowing; safe() still checks commanded and measured motion.
   */
  double translation_limit() const
  {
    double limit = steps_.empty() ? speed_ : std::min(speed_, steps_[index_ - 1].max_speed);
    const double distance = std::hypot(tx_ - x_, ty_ - y_);
    limit = std::min(limit, .12 + .6 * std::max(0.0, distance - .12));
    const double preview = std::min(distance, .12) / std::max(distance, 1e-9);
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    const double dx = preview * (c * (tx_ - x_) + s * (ty_ - y_));
    const double dy = preview * (-s * (tx_ - x_) + c * (ty_ - y_));
    if (maze::translation_clearance(points_, dx, dy) < .04) limit = std::min(limit, .12);
    return limit;
  }

  /** @brief Compute translational PID in odom then rotate output into body coordinates.
  *
 * @param[in] dt Positive ROS seconds from tick().
 * @param[in,out] cmd Writes linear x/y,
  * preserving the heading controller's angular z; correction and guard consume it next. */
  void move(double dt, geometry_msgs::msg::Twist & cmd)
  {
    const double c = std::cos(yaw_), s = std::sin(yaw_), limit = translation_limit();
    double wx = px_->update(tx_ - x_, c * vx_ - s * vy_, dt, limit),
           wy = py_->update(ty_ - y_, s * vx_ + c * vy_, dt, limit);
    const double scale = std::min(1.0, limit / std::max(1e-9, std::hypot(wx, wy)));
    cmd.linear.x = scale * (c * wx + s * wy);
    cmd.linear.y = scale * (-s * wx + c * wy);
    correct(cmd);
    const double corrected_scale =
      std::min(1.0, limit / std::max(1e-9, std::hypot(cmd.linear.x, cmd.linear.y)));
    cmd.linear.x *= corrected_scale;
    cmd.linear.y *= corrected_scale;
  }

  /** @brief Recover turning clearance by a bounded translation away from the nearest wall.
   *
 * @note Called only after a blocked TURN, with no rotation commanded; abort beyond 6 cm.
   *
 * @return True if a separately guarded clearance command was published, false otherwise.
   */
  bool recover_turn_clearance()
  {
    if (final_turn_active_ || moving_ || std::abs(wz_) > .02 || std::hypot(vx_, vy_) > .03)
      return false;
    if (std::hypot(x_ - turn_x_, y_ - turn_y_) > .06) {
      fail("TURN_CLEARANCE_LIMIT");
      return false;
    }
    auto point =
      std::min_element(points_.begin(), points_.end(), [](const auto & a, const auto & b) {
        return maze::clearance(a[0], a[1]) < maze::clearance(b[0], b[1]);
      });
    if (point == points_.end()) return false;
    double norm = std::hypot((*point)[0], (*point)[1]);
    geometry_msgs::msg::Twist escape;
    escape.linear.x = -.02 * (*point)[0] / norm;
    escape.linear.y = -.02 * (*point)[1] / norm;
    if (!safe(escape, true)) return false;
    pub_->publish(escape);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "CLEARANCE adjustment before TURN P%02zu", index_ + 1);
    return true;
  }

  /** @brief Start a clockwise half-turn from the actual stopped P15 heading.
   * @par Terminal action
   * complete_stage() calls this only after the complete route reaches its final point.
   * odom() accumulates accepted signed yaw increments; tick() reads the remaining rotation.
   * @note No new XY waypoint, initialization or translation is requested. Obstacle recovery
   * translation is disabled for this action; blocked rotation still stops and faults.
   */
  void begin_final_turn()
  {
    final_turn_active_ = true;
    final_rotation_ = 0.0;
    moving_ = false;
    heading_ = yaw_ - M_PI;
    reset();
    stage_started_ = Clock::now();
    stage_ros_started_ = now();
    RCLCPP_INFO(
      get_logger(), "Final clockwise turn: start=%.6f target=%.6f delta=-3.141593 rad", yaw_,
      heading_);
  }

  /** @brief Complete a stopped TURN or MOVE and select the next state.
   *
 * @param[in] distance Current odom position error from tick(), meters, used in arrival logging.
   *
 * @param[in] e Current heading error from tick(), radians, used in arrival logging.
   *
 * @note Writes moving_, index_ and targets; final waypoint stops and shuts down ROS.
   */
  void complete_stage(double distance, double e)
  {
    stop();
    const bool ready = settled(true);
    log_turn_settling(e);
    if (!ready) return;
    if (final_turn_active_) {
      RCLCPP_INFO(
        get_logger(), "Final clockwise turn completed: rotation=%.6f error=%.6f rad",
        final_rotation_, e);
      RCLCPP_INFO(get_logger(), "Route completed; stopped.");
      rclcpp::shutdown();
      return;
    }
    if (!moving_) {
      moving_ = true;
      reset();
      stage_started_ = Clock::now();
      stage_ros_started_ = now();
      RCLCPP_INFO(get_logger(), "P%02zu MOVE", index_ + 1);
    } else {
      RCLCPP_INFO(
        get_logger(), "Reached P%02zu: distance=%.6f heading_error=%.6f", index_ + 1, distance, e);
      if (index_ + 1 >= static_cast<std::size_t>(last_index_)) {
        if (final_turn_enabled_ && static_cast<std::size_t>(last_index_) == route_.size()) {
          begin_final_turn();
          return;
        }
        stop();
        RCLCPP_INFO(get_logger(), "Route completed; stopped.");
        rclcpp::shutdown();
        return;
      }
      ++index_;
      target(true);
    }
  }

  /** @brief Advance TURN/MOVE/arrival states, enforcing all feedback and obstacle guards. */
  void tick()
  {
    if (!initialized_) {
      initialize();
      return;
    }
    if (elapsed(odom_received_) > .5 || elapsed(scan_received_) > .5) {
      fail("FEEDBACK_TIMEOUT");
      return;
    }
    if (elapsed(stage_started_) > stage_timeout_) {
      fail("STAGE_TIMEOUT");
      return;
    }
    const double dt = (now() - last_).seconds();
    if (dt == 0) return;
    if (dt < 0 || dt > .5) {
      fail("CLOCK_JUMP");
      return;
    }
    last_ = now();
    center_from_sides();
    const double e = final_turn_active_ ? -M_PI - final_rotation_ : maze::wrap(heading_ - yaw_);
    const double distance = std::hypot(tx_ - x_, ty_ - y_);
    bool close = std::abs(e) < .01 && (!moving_ || distance < .015);
    if (close) {
      complete_stage(distance, e);
      return;
    }
    holding_ = false;
    log_turn_settling(e);
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = yaw_pid_->update(e, wz_, dt);
    if (moving_ && std::abs(e) < .15) move(dt, cmd);
    if (!safe(cmd)) {
      stop();
      px_->reset();
      py_->reset();
      yaw_pid_->reset();
      if (!blocked_) {
        blocked_ = true;
        blocked_since_ = Clock::now();
        RCLCPP_WARN(get_logger(), "OBSTACLE_HOLD P%02zu", index_ + 1);
      }
      if (recover_turn_clearance()) return;
      if (elapsed(blocked_since_) > 5) fail("OBSTACLE_BLOCKED");
      return;
    }
    blocked_ = false;
    pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "P%02zu %s distance=%.3f yaw_error=%.3f cmd=(%.3f,%.3f,%.3f)", index_ + 1,
      final_turn_active_ ? "FINAL_TURN" : (moving_ ? "MOVE" : "TURN"), distance, e, cmd.linear.x,
      cmd.linear.y, cmd.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;        ///< Sole velocity publisher.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  ///< Advancing pose feedback.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  ///< Wall observations.
  rclcpp::TimerBase::SharedPtr timer_;         ///< 20ms steady timer, ROS-time PID updates.
  tf2_ros::Buffer buffer_;                     ///< Scan-to-body transforms.
  tf2_ros::TransformListener listener_;        ///< TF subscription owner.
  std::vector<std::array<double, 2>> points_;  ///< Accepted laser points in base_link, meters.
  std::vector<maze::Point> route_;             ///< Generated nominal route poses relative to P01.
  std::vector<maze::MotionStep>
    steps_;  ///< Relative actions; empty in fixed-point compatibility mode.
  bool
    final_turn_enabled_{};  ///< Append clockwise 180 degrees after the full route, not last_point trials.
  bool final_turn_active_{};  ///< Terminal rotation only; no MOVE or clearance translation.
  double
    final_rotation_{};  ///< Signed odom yaw accumulated since terminal entry, radians; clockwise negative.
  bool side_centering_{};  ///< Enables gated bilateral wall centering for forward motion.
  int center_count_{};     ///< Consecutive accepted fresh side-wall fits in this segment.
  rclcpp::Time center_scan_{0, 0, RCL_ROS_TIME};  ///< Last scan consumed for centering.
  double center_offset_{};  ///< Accepted cross-track target shift, meters, bounded to +/-0.06.
  double nominal_tx_{};     ///< Generated odom target x before wall correction, meters.
  double nominal_ty_{};     ///< Generated odom target y before wall correction, meters.
  std::string frame_;       ///< Odom frame retained to detect frame changes.
  std::unique_ptr<maze::AxisPid> px_;        ///< Odom x PID; state reset at each stage.
  std::unique_ptr<maze::AxisPid> py_;        ///< Odom y PID; state reset at each stage.
  std::unique_ptr<maze::AxisPid> yaw_pid_;   ///< Heading PID; radian error and body yaw-rate input.
  Clock::time_point started_{Clock::now()};  ///< Steady startup deadline base.
  Clock::time_point odom_received_{};        ///< Steady time of last accepted odom.
  Clock::time_point scan_received_{};        ///< Steady time of last accepted scan.
  rclcpp::Clock diagnostic_clock_{
    RCL_STEADY_TIME};  ///< Throttles turn diagnostics in real elapsed time.
  rclcpp::Time stage_ros_started_{
    0, 0, RCL_ROS_TIME};               ///< ROS-time counterpart of stage_started_, for logs only.
  Clock::time_point stage_started_{};  ///< Steady TURN/MOVE deadline base.
  Clock::time_point blocked_since_{};  ///< Steady start of continuous obstacle hold.
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};  ///< Last accepted odom ROS stamp.
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};  ///< Last accepted scan ROS stamp.
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};        ///< Last PID update ROS time.
  rclcpp::Time hold_{0, 0, RCL_ROS_TIME};        ///< Stopped qualification start ROS time.
  double turn_x_{};         ///< Odom x at TURN entry, meters; bounds local clearance recovery.
  double turn_y_{};         ///< Odom y at TURN entry, meters; bounds local clearance recovery.
  double x_{};              ///< Current odom x, meters.
  double y_{};              ///< Current odom y, meters.
  double yaw_{};            ///< Current odom heading, radians.
  double vx_{};             ///< Measured body x velocity, m/s.
  double vy_{};             ///< Measured body y velocity, m/s.
  double wz_{};             ///< Measured body yaw rate, rad/s.
  double ox_{};             ///< Frozen P01 odom x, meters.
  double oy_{};             ///< Frozen P01 odom y, meters.
  double oyaw_{};           ///< Frozen route heading, radians.
  double tx_{};             ///< Fixed destination odom x, meters.
  double ty_{};             ///< Fixed destination odom y, meters.
  double heading_{};        ///< Fixed destination odom yaw, radians.
  double speed_{};          ///< Planar PID speed bound, m/s.
  double accel_{};          ///< Planar PID slew bound, m/s squared.
  double stop_hold_{};      ///< Required stopped pose hold in ROS seconds; excludes startup.
  double stage_timeout_{};  ///< Steady stage deadline, seconds.
  int last_index_{15};      ///< One-based final route point.
  int code_{};              ///< Process failure code; zero on normal completion.
  std::size_t index_{1};    ///< Zero-based active destination; P01 is the origin.
  bool have_odom_{};        ///< True after accepting at least one odom message.
  bool have_scan_{};        ///< True after accepting a TF-transformed scan.
  bool initialized_{};      ///< True once the route origin has been frozen.
  bool moving_{};           ///< True for MOVE, false for TURN.
  bool holding_{};          ///< True while continuously qualifying stopped pose.
  bool blocked_{};          ///< True while stopped by the obstacle guard.
};

/** @brief Own controller lifetime and return its failure status.
 *
 * @param[in] argc Shell argument count passed to ROS.
 *
 * @param[in] argv ROS arguments read by initialization.
 *
 * @return Zero on final completion, two on guarded failure, one on configuration error. */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto n = std::make_shared<PIDMazeSolver>();
    rclcpp::spin(n);
    n->stop();
    if (rclcpp::ok()) rclcpp::shutdown();
    return n->result();
  } catch (const std::exception & e) {
    fprintf(stderr, "%s\n", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
