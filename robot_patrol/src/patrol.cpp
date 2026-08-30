#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

class AutonomousExplorationNode : public rclcpp::Node {
public:
  AutonomousExplorationNode() : Node("autonomous_exploration_node") {
    // Subscriber to LaserScan
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&AutonomousExplorationNode::laserscan_callback, this,
                  std::placeholders::_1));

    // Publisher for movement commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    // Initialize state variables
    turning_ = false;
    turn_direction_ = -0.5; // Default to turning right

    RCLCPP_INFO(this->get_logger(), "Autonomous Exploration Node Ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Define the sectors
    std::map<std::string, std::pair<int, int>> sectors = {
        {"Front_Left", {0, 11}},
        {"Left", {12, 50}},
        {"Right", {150, 187}},
        {"Front_Right", {188, 199}}};

    // Initialize the minimum distances for each sector
    std::map<std::string, float> min_distances;
    std::map<std::string, float> max_distances;
    for (const auto &sector : sectors) {
      min_distances[sector.first] = std::numeric_limits<float>::infinity();
    }

    // Find the minimum distance in each sector
    for (const auto &sector : sectors) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;

      // Ensure the index range is within bounds and not empty
      if (start_idx < static_cast<int>(msg->ranges.size()) &&
          end_idx < static_cast<int>(msg->ranges.size())) {
        float min_val = std::numeric_limits<float>::infinity();
        float max_val = -1.0f;
        for (int i = start_idx; i <= end_idx; ++i) {
          float range = msg->ranges[i];
          if (std::isinf(range) || std::isnan(range) ||
              range < msg->range_min || range > msg->range_max) {
            continue;
          }

          if (range < min_val) {
            min_val = range;
          }
          if (range > max_val) {
            max_val = range;
          }
        }
        min_distances[sector.first] = min_val;
        max_distances[sector.first] = max_val;
      }
    }

    // Define the threshold for obstacle detection
    float obstacle_threshold = 0.35; // meters

    // Narrow front check: only Front_Left and Front_Right determine
    // whether an obstacle is directly in front
    float min_narrow =
        std::min(min_distances["Front_Left"], min_distances["Front_Right"]);

    // Safest direction: side containing the single greatest valid ray
    float left_max =
        std::max(max_distances["Front_Left"], max_distances["Left"]);
    float right_max =
        std::max(max_distances["Front_Right"], max_distances["Right"]);
    float turn_direction = (left_max >= right_max) ? 1.0f : -1.0f;

    auto action = geometry_msgs::msg::Twist();

    if (min_narrow >= obstacle_threshold) {
      action.linear.x = 0.1;
      action.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Path clear (%.2f m). Moving forward.",
                  min_narrow);
    } else {
      action.linear.x = 0.05;
      action.angular.z = 0.5 * turn_direction;
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle at %.2f m in front. Turning %s.", min_narrow,
                  turn_direction > 0 ? "LEFT" : "RIGHT");
    }

    publisher_->publish(action);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  bool turning_;
  double turn_direction_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousExplorationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}