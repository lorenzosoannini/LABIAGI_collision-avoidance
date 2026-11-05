#include "collision_avoidance/controller.hpp"
#include <fstream>
#include <iostream>
#include <string>

using namespace collision_avoidance;
constexpr double pi = 3.14159265358979323846;
struct Circle { double x, y, r; };
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
std::vector<Point> scan(double x, double y, double yaw, const std::vector<Circle>& circles) {
  std::vector<Point> points;
  for (int i = 0; i < 360; ++i) {
    const double angle = 2.0 * pi * i / 360.0;
    const double dx = std::cos(yaw + angle), dy = std::sin(yaw + angle);
    double hit = 8.0;
    for (const auto& c : circles) {
      const double px = c.x - x, py = c.y - y;
      const double projection = px * dx + py * dy;
      const double discriminant = c.r * c.r - (px * px + py * py - projection * projection);
      if (discriminant < 0.0) continue;
      const double near = projection - std::sqrt(discriminant);
      if (near >= 0.0) hit = std::min(hit, near);
    }
    if (hit < 8.0) points.push_back({hit * std::cos(angle), hit * std::sin(angle)});
  }
  return points;
}
int main(int argc, char** argv) {
  const std::string scenario = argc > 2 ? argv[2] : "single";
  std::vector<Circle> circles = {{2.5, 0.0, 0.45}};
  if (scenario == "offset") circles = {{2.5, 0.25, 0.45}};
  if (scenario == "multiple") circles = {{2.3, 0.0, 0.40}, {4.4, -0.25, 0.45}};
  if (scenario == "clear") circles.clear();
  if (scenario != "single" && scenario != "offset" && scenario != "multiple" && scenario != "clear") return 2;
  std::ofstream file(argc > 1 ? argv[1] : "trajectory.csv");
  if (!file) return 2;
  file << "t,x,y,yaw,desired_v,desired_w,v,w,clearance\n";
  Controller controller;
  double x = 0.0, y = 0.0, yaw = 0.0, min_gap = 100.0, deviation = 0.0;
  const double goal_x = 6.5, dt = 0.10;
  int i = 0;
  for (; i < 800 && std::hypot(goal_x - x, y) > 0.15; ++i) {
    // External intent provider: steer toward a goal. The avoidance filter gets no goal.
    Command desired;
    desired.linear = std::min(0.55, std::hypot(goal_x - x, y));
    desired.angular = std::max(-1.2, std::min(1.2, 1.6 * wrap(std::atan2(-y, goal_x - x) - yaw)));
    Command cmd = controller.filter(desired, scan(x, y, yaw, circles));
    const Point delta = Controller::endpoint(cmd, dt);
    x += std::cos(yaw) * delta.x - std::sin(yaw) * delta.y;
    y += std::sin(yaw) * delta.x + std::cos(yaw) * delta.y;
    yaw = wrap(yaw + cmd.angular * dt);
    double gap = 8.0;
    for (const auto& c : circles) gap = std::min(gap, std::hypot(x - c.x, y - c.y) - c.r - 0.20);
    min_gap = std::min(min_gap, gap);
    deviation = std::max(deviation, std::abs(y));
    file << i * dt << ',' << x << ',' << y << ',' << yaw << ',' << desired.linear << ',' << desired.angular
         << ',' << cmd.linear << ',' << cmd.angular << ',' << gap << '\n';
    if (gap < 0.099) { std::cerr << "Clearance violation\n"; return 1; }
  }
  std::cout << scenario << ": steps=" << i << " goal_distance=" << std::hypot(goal_x-x,y)
            << " min_surface_clearance=" << min_gap << " max_deviation=" << deviation << '\n';
  return i == 800 ? 1 : 0;
}
