#include "collision_avoidance/controller.hpp"
#include "collision_avoidance/watchdog.hpp"
#include <cstdlib>
#include <iostream>
using namespace collision_avoidance;
void require(bool value, const char* message) {
  if (!value) { std::cerr << message << '\n'; std::exit(1); }
}
Command command(double v, double w) { Command c; c.linear=v; c.angular=w; return c; }
int main() {
  Controller c;
  auto result = c.filter(command(0.5, 0.2), {});
  require(result.linear == 0.5 && result.angular == 0.2, "Clear commands must pass through");
  std::vector<Point> front;
  for (int i=-8; i<=8; ++i) front.push_back({1.0, i*0.025});
  result = c.filter(command(0.5, 0), front);
  require(result.linear > 0.0 && std::abs(result.angular) > 0.1, "Must steer around a frontal obstacle, not just stop");
  std::vector<Point> ring;
  for (int i=0; i<72; ++i) {
    const double angle = i * 6.283185307179586 / 72.0;
    ring.push_back({.35 * std::cos(angle), .35 * std::sin(angle)});
  }
  result = c.filter(command(.5, 0), ring);
  require(result.linear == 0.0, "Enclosed robot must not advance");
  result = c.filter(command(0, 0), front);
  require(result.linear == 0 && result.angular == 0, "Zero input must stop");
  result = c.filter(command(-0.4, 0), {});
  require(result.linear == 0 && result.angular == 0, "Unsupported reverse must stop");
  result = c.filter(command(0.5, 0), {{0.10, 0.0}});
  require(result.linear == 0 && result.angular == 0, "Intrusion into footprint must stop");
  result = c.filter(command(100, 100), {});
  require(result.linear <= .6 && std::abs(result.angular) <= 1.5, "Output must be bounded");
  result = c.filter(command(std::numeric_limits<double>::quiet_NaN(), 0), {});
  require(result.linear == 0, "Nonfinite command must stop");
  result = c.filter(command(.5, 0), {{std::numeric_limits<double>::infinity(), 0}});
  require(result.linear == 0, "Nonfinite geometry must stop");
  Config bad; bad.step = 0.0;
  bool rejected=false; try { Controller invalid(bad); } catch(const std::invalid_argument&) { rejected=true; }
  require(rejected, "Invalid configuration must be rejected");
  require(inputsFresh(.1, .1, true, .5, .3), "Fresh inputs should pass");
  require(!inputsFresh(.6, .1, true, .5, .3), "Stale command must stop");
  require(!inputsFresh(.1, .4, true, .5, .3), "Stale scan must stop");
  require(!inputsFresh(.1, .1, false, .5, .3), "Failed scan must stop");
  require(!inputsFresh(-.1, .1, true, .5, .3), "Negative age must stop");
  std::cout << "Controller tests passed\n";
}
