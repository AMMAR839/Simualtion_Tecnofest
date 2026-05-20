#include "gamantaray_nav2_bt_plugins/asv_obstacle_window_condition.hpp"

#include <cmath>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace gamantaray_nav2_bt_plugins
{

AsvObstacleWindowCondition::AsvObstacleWindowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

void AsvObstacleWindowCondition::initialize()
{
  getInput("scan_topic", scan_topic_);
  getInput("max_scan_age", max_scan_age_);
  getInput("min_range", min_range_);
  getInput("forward_distance", forward_distance_);
  getInput("half_width", half_width_);
  getInput("max_points_in_window", max_points_in_window_);

  auto qos = rclcpp::SensorDataQoS();
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, qos,
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      onScan(msg);
    },
    sub_options);

  initialized_ = true;
  last_log_time_ = node_->now();
}

void AsvObstacleWindowCondition::onScan(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = msg;
}

BT::NodeStatus AsvObstacleWindowCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  callback_group_executor_.spin_some();

  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }

  const auto now = node_->now();
  if (!scan) {
    if ((now - last_log_time_).seconds() > 2.0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ASV BT guard has not received scan topic %s", scan_topic_.c_str());
      last_log_time_ = now;
    }
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time scan_time(scan->header.stamp);
  if ((now - scan_time).seconds() > max_scan_age_) {
    if ((now - last_log_time_).seconds() > 2.0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ASV BT guard scan is stale: age %.2fs", (now - scan_time).seconds());
      last_log_time_ = now;
    }
    return BT::NodeStatus::FAILURE;
  }

  const int points = countWindowPoints(*scan);
  if (points > max_points_in_window_) {
    if ((now - last_log_time_).seconds() > 2.0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ASV BT guard detected dense local clutter: %d points in %.1fm x %.1fm window",
        points, forward_distance_, half_width_ * 2.0);
      last_log_time_ = now;
    }
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

int AsvObstacleWindowCondition::countWindowPoints(
  const sensor_msgs::msg::LaserScan & scan) const
{
  int count = 0;
  double angle = scan.angle_min;
  const double effective_min_range = std::max(
    static_cast<double>(scan.range_min), min_range_);
  for (const auto distance : scan.ranges) {
    if (std::isfinite(distance) &&
      distance >= effective_min_range &&
      distance <= scan.range_max)
    {
      const double x = distance * std::cos(angle);
      const double y = distance * std::sin(angle);
      if (x > 0.0 && x <= forward_distance_ && std::abs(y) <= half_width_) {
        ++count;
      }
    }
    angle += scan.angle_increment;
  }
  return count;
}

}  // namespace gamantaray_nav2_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gamantaray_nav2_bt_plugins::AsvObstacleWindowCondition>(
    "AsvObstacleWindow");
}
