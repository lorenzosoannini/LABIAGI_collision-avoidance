#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace collision_avoidance {
struct Point { double x, y; };
struct Command { double linear = 0.0, angular = 0.0; };
struct Config {
  double robot_radius = 0.20;
  double safety_margin = 0.10;
  double max_linear = 0.60;
  double max_angular = 1.50;
  double horizon = 2.50;
  double step = 0.10;
  double braking_deceleration = 0.80;
  double reaction_time = 0.40;
  void validate() const {
    for (double v : {robot_radius, safety_margin, max_linear, max_angular,
                     horizon, step, braking_deceleration, reaction_time})
      if (!std::isfinite(v) || v <= 0.0) throw std::invalid_argument("Controller parameters must be finite and positive");
    if (step > horizon || horizon > 10.0 || horizon / step > 500.0)
      throw std::invalid_argument("Invalid rollout resolution or horizon");
  }
};

// Local velocity filter for a circular differential-drive robot. No global goal/map.
// Points must be current obstacle returns expressed in the robot's base frame.
class Controller {
 public:
  explicit Controller(Config config = Config()) : cfg_(config) { cfg_.validate(); }
  Command filter(Command desired, const std::vector<Point>& obstacles, double sensor_age = 0.0) {
    if (!std::isfinite(sensor_age) || sensor_age < 0.0) return stop();
    sensor_age_ = sensor_age;
    if (!std::isfinite(desired.linear) || !std::isfinite(desired.angular) ||
        desired.linear < 0.0) return stop();  // Reverse is deliberately unsupported.
    desired.linear = std::min(desired.linear, cfg_.max_linear);
    desired.angular = clamp(desired.angular, -cfg_.max_angular, cfg_.max_angular);
    for (const auto& p : obstacles)
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) return stop();
    if (desired.linear < 1e-6 && std::abs(desired.angular) < 1e-6) return stop();

    // Preserve clear user commands exactly; avoid adding steering in free space.
    const double desired_gap = clearance(desired, obstacles);
    if (desired_gap > 0.25 || (desired.linear < 1e-6 && desired_gap > 0.0)) { previous_ = desired; side_ = 0; return desired; }
    if (side_ == 0) {
      double lateral = 0.0;
      for (const auto& p : obstacles)
        if (p.x > 0.0 && p.x < cfg_.max_linear * cfg_.horizon + 0.5)
          lateral += p.y / std::max(0.1, p.x * p.x + p.y * p.y);
      side_ = lateral > 0.05 ? -1 : 1;
    }
    double best = std::numeric_limits<double>::infinity();
    Command selected;
    // Include the requested arc as well as sampled alternatives, including rotation.
    for (int speed = 0; speed <= 4; ++speed) {
      const double v = desired.linear * speed / 4.0;
      for (int turn = -15; turn <= 16; ++turn) {
        const double w = turn == 16 ? desired.angular : cfg_.max_angular * turn / 15.0;
        if (w * side_ < -1e-6) continue;  // Keep the bypass side until the desired arc clears.
        Command candidate; candidate.linear = v; candidate.angular = w;
        const double gap = clearance(candidate, obstacles);
        if (gap <= 0.0) continue;
        // Endpoint tracking approximates the operator's intended local motion.
        const Point target = endpoint(desired, cfg_.horizon);
        const Point end = endpoint(candidate, cfg_.horizon);
        double cost = 2.0 * std::hypot(end.x - target.x, end.y - target.y)
                    + 0.25 * std::abs(w - desired.angular)
                    + 0.12 * std::abs(w - previous_.angular)
                    + 0.12 / (gap + 0.10) - 1.0 * v;
        // Break exact symmetry consistently to the left, without a large turn bias.
        if (desired.linear > 1e-6) cost += 3.0 * (1.0 - v / desired.linear);
        cost -= 1e-5 * w;
        if (cost < best) { best = cost; selected = candidate; }
      }
    }
    previous_ = selected;
    return selected;
  }
  Command stop() { side_ = 0; previous_ = Command(); return previous_; }
  static Point endpoint(Command command, double t) {
    if (std::abs(command.angular) < 1e-8) return {command.linear * t, 0.0};
    const double radius = command.linear / command.angular;
    return {radius * std::sin(command.angular * t), radius * (1.0 - std::cos(command.angular * t))};
  }
 private:
  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
  double clearance(Command command, const std::vector<Point>& points) const {
    double gap = 10.0;
    // Inflating by half a rollout step bounds inter-sample translation distance.
    const double inflation = cfg_.robot_radius + cfg_.safety_margin + command.linear * cfg_.step / 2.0;
    for (double t = 0.0; t < cfg_.horizon + cfg_.step; t += cfg_.step) {
      const Point pos = endpoint(command, std::min(t, cfg_.horizon));
      for (const auto& p : points) {
        gap = std::min(gap, std::hypot(p.x - pos.x, p.y - pos.y) - inflation - uncertainty(p));
        if (gap <= 0.0) return gap;
      }
    }
    // A command must leave enough initial straight-line space to brake.
    const double stop_distance = command.linear * cfg_.reaction_time +
        command.linear * command.linear / (2.0 * cfg_.braking_deceleration);
    for (const auto& p : points)
      if (p.x > 0.0 && std::abs(p.y) < inflation + uncertainty(p) && p.x - inflation - uncertainty(p) < stop_distance)
        return -1.0;
    return gap;
  }
  double uncertainty(const Point& point) const {
    // Bound motion of a base-frame return since acquisition: translation + rotation.
    return cfg_.max_linear * sensor_age_ + std::hypot(point.x, point.y) *
        std::min(2.0, cfg_.max_angular * sensor_age_);
  }
  Config cfg_;
  Command previous_;
  int side_ = 0;
  double sensor_age_ = 0.0;
};
}  // namespace collision_avoidance
