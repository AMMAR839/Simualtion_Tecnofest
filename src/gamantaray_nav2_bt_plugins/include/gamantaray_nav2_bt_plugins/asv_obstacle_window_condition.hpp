#ifndef GAMANTARAY_NAV2_BT_PLUGINS__ASV_OBSTACLE_WINDOW_CONDITION_HPP_
#define GAMANTARAY_NAV2_BT_PLUGINS__ASV_OBSTACLE_WINDOW_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace gamantaray_nav2_bt_plugins
{

class AsvObstacleWindowCondition : public BT::ConditionNode
{
public:
  AsvObstacleWindowCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  AsvObstacleWindowCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("scan_topic", "/asv/lidar/scan", "Filtered LaserScan topic"),
      BT::InputPort<double>("max_scan_age", 0.9, "Maximum accepted scan age in seconds"),
      BT::InputPort<double>("min_range", 0.75, "Minimum valid obstacle range"),
      BT::InputPort<double>("forward_distance", 5.0, "Forward window length in meters"),
      BT::InputPort<double>("half_width", 1.5, "Half width of the forward window"),
      BT::InputPort<int>("max_points_in_window", 120, "Point count treated as wave/noise clutter")
    };
  }

private:
  void initialize();
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  int countWindowPoints(const sensor_msgs::msg::LaserScan & scan) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  mutable std::mutex scan_mutex_;

  bool initialized_{false};
  std::string scan_topic_;
  double max_scan_age_{0.9};
  double min_range_{0.75};
  double forward_distance_{5.0};
  double half_width_{1.5};
  int max_points_in_window_{120};
  rclcpp::Time last_log_time_;
};

}  // namespace gamantaray_nav2_bt_plugins

#endif  // GAMANTARAY_NAV2_BT_PLUGINS__ASV_OBSTACLE_WINDOW_CONDITION_HPP_
