#include "collision_avoidance/controller.hpp"
#include "collision_avoidance/watchdog.hpp"
#include <geometry_msgs/Twist.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <memory>
#include <string>

namespace ca = collision_avoidance;
class AvoidanceNode {
 public:
  AvoidanceNode() : private_("~") {
    ca::Config cfg;
    private_.param("robot_radius", cfg.robot_radius, cfg.robot_radius);
    private_.param("safety_margin", cfg.safety_margin, cfg.safety_margin);
    private_.param("max_linear", cfg.max_linear, cfg.max_linear);
    private_.param("max_angular", cfg.max_angular, cfg.max_angular);
    private_.param("horizon", cfg.horizon, cfg.horizon);
    private_.param("rollout_step", cfg.step, cfg.step);
    private_.param("braking_deceleration", cfg.braking_deceleration, cfg.braking_deceleration);
    private_.param("reaction_time", cfg.reaction_time, 0.40);
    private_.param("control_rate", control_rate_, 20.0);
    private_.param("command_timeout", command_timeout_, 0.5);
    private_.param("scan_timeout", scan_timeout_, 0.3);
    private_.param<std::string>("base_frame", base_frame_, "base_link");
    for (double v : {control_rate_, command_timeout_, scan_timeout_})
      if (!std::isfinite(v) || v <= 0.0) throw std::invalid_argument("Rates/timeouts must be finite and positive");
    if (control_rate_ > 200.0 || cfg.reaction_time < scan_timeout_ + 1.0 / control_rate_)
      throw std::invalid_argument("reaction_time must cover scan_timeout + one control period; rate <= 200 Hz");
    controller_.reset(new ca::Controller(cfg));
    // Account for a scanner offset when requiring enough range for the rollout.
    required_range_ = cfg.max_linear * cfg.horizon + cfg.robot_radius + cfg.safety_margin;
    std::string input, output, scan;
    private_.param<std::string>("input_topic", input, "cmd_vel_raw");
    private_.param<std::string>("output_topic", output, "cmd_vel");
    private_.param<std::string>("scan_topic", scan, "base_scan");
    if (node_.resolveName(input) == node_.resolveName(output))
      throw std::invalid_argument("Input and output velocity topics must be different");
    publisher_ = node_.advertise<geometry_msgs::Twist>(output, 1);
    command_sub_ = node_.subscribe(input, 1, &AvoidanceNode::commandCallback, this);
    scan_sub_ = node_.subscribe(scan, 1, &AvoidanceNode::scanCallback, this);
    // A wall timer still sends stop commands when simulation time is paused.
    timer_ = node_.createWallTimer(ros::WallDuration(1.0/control_rate_), &AvoidanceNode::tick, this);
  }
 private:
  void commandCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    command_stamp_ = ros::WallTime::now();
    command_.linear = msg->linear.x;
    command_.angular = msg->angular.z;
    // A differential-drive filter cannot execute lateral/vertical/roll/pitch commands.
    if (!std::isfinite(msg->linear.y) || !std::isfinite(msg->linear.z) ||
        !std::isfinite(msg->angular.x) || !std::isfinite(msg->angular.y) ||
        std::abs(msg->linear.y) > 1e-6 || std::abs(msg->linear.z) > 1e-6 ||
        std::abs(msg->angular.x) > 1e-6 || std::abs(msg->angular.y) > 1e-6)
      command_ = ca::Command();
  }
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    scan_valid_ = false;  // A malformed scan immediately invalidates previous geometry.
    const double span = msg->angle_increment * (msg->ranges.empty() ? 0 : msg->ranges.size()-1);
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    if (msg->ranges.size() < 2 || msg->header.frame_id.empty() || msg->header.stamp.isZero() ||
        !std::isfinite(age) || age < -0.05 || age > scan_timeout_ ||
        !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_max) ||
        !std::isfinite(msg->angle_increment) || msg->angle_increment <= 0.0 || span < 6.18 || span > 6.39 ||
        std::abs((msg->angle_max-msg->angle_min)-span) > 0.02 ||
        !std::isfinite(msg->time_increment) || msg->time_increment < 0.0 ||
        !std::isfinite(msg->range_min) || msg->range_min < 0.0 ||
        !std::isfinite(msg->range_max) || msg->range_max <= msg->range_min) {
      ROS_WARN_THROTTLE(2.0, "Rejecting stale/malformed scan; near-360-degree coverage required");
      return;
    }
    for (float range : msg->ranges) {
      if (std::isnan(range) || range == -std::numeric_limits<float>::infinity() ||
          (std::isfinite(range) && range < msg->range_min)) {
        ROS_WARN_THROTTLE(2.0, "Invalid/too-close laser return: stopping");
        return;
      }
    }
    try {
      const auto transform = [&]() {
        tf::StampedTransform value;
        listener_.lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp, value);
        return value;
      }();
      if (msg->range_max < required_range_ + transform.getOrigin().length()) {
        ROS_WARN_THROTTLE(2.0, "Laser range is too short for the configured rollout");
        return;
      }
      sensor_msgs::PointCloud cloud;
      projector_.transformLaserScanToPointCloud(base_frame_, *msg, cloud, listener_);
      points_.clear();
      for (const auto& p : cloud.points) points_.push_back({p.x, p.y});
      scan_stamp_ = ros::WallTime::now();
      // Preserve acquisition age as well as receipt age in the watchdog.
      scan_acquisition_age_ = std::max(0.0, age);
      scan_valid_ = true;
    } catch (const tf::TransformException& e) {
      ROS_WARN_THROTTLE(2.0, "Laser transform unavailable: %s", e.what());
    }
  }
  void tick(const ros::WallTimerEvent&) {
    const auto now = ros::WallTime::now();
    ca::Command filtered;
    if (ca::inputsFresh((now-command_stamp_).toSec(), (now-scan_stamp_).toSec()+scan_acquisition_age_,
                        scan_valid_, command_timeout_, scan_timeout_))
      filtered = controller_->filter(command_, points_);
    else controller_->stop();
    geometry_msgs::Twist msg;
    msg.linear.x = filtered.linear;
    msg.angular.z = filtered.angular;
    publisher_.publish(msg);
  }
  ros::NodeHandle node_, private_;
  ros::Subscriber command_sub_, scan_sub_;
  ros::Publisher publisher_;
  ros::WallTimer timer_;
  tf::TransformListener listener_;
  laser_geometry::LaserProjection projector_;
  std::unique_ptr<ca::Controller> controller_;
  std::vector<ca::Point> points_;
  ca::Command command_;
  ros::WallTime command_stamp_, scan_stamp_;
  bool scan_valid_ = false;
  double control_rate_, command_timeout_, scan_timeout_, required_range_, scan_acquisition_age_ = 0.0;
  std::string base_frame_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_avoidance");
  try { AvoidanceNode node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("Cannot start avoidance node: %s", e.what()); return 1; }
  return 0;
}
