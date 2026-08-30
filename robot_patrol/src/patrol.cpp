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
        {"Right_Rear", {0, 33}},    {"Right", {34, 66}},
        {"Front_Right", {67, 100}}, {"Front_Left", {101, 133}},
        {"Left", {134, 166}},       {"Left_Rear", {167, 199}}};

    // Initialize the minimum distances for each sector
    std::map<std::string, float> min_distances;
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
        for (int i = start_idx; i <= end_idx; ++i) {
          if (msg->ranges[i] < min_val) {
            min_val = msg->ranges[i];
          }
        }
        min_distances[sector.first] = min_val;
      }
    }

    // Define the threshold for obstacle detection
    float obstacle_threshold = 0.35; // meters

    // Determine detected obstacles
    std::map<std::string, bool> detections;
    for (const auto &min_dist : min_distances) {
      detections[min_dist.first] = min_dist.second < obstacle_threshold;
    }

    // Determine suggested action based on detection
    auto action = geometry_msgs::msg::Twist();

    // If obstacles are detected in both front sectors, continue turning
    if (detections["Front_Left"] || detections["Front_Right"]) {
      if (!turning_) {
        // Start turning if not already turning
        turning_ = true;
        turn_direction_ = -0.5; // Turning right
      }
      action.angular.z = turn_direction_; // Continue turning
      RCLCPP_INFO(this->get_logger(), "Obstacle ahead, turning to clear path.");
    } else {
      turning_ = false; // Stop turning when the front is clear
      // Priority 2: Side detections
      if (detections["Left"]) {
        action.linear.x = 0.2;   // Move forward slowly
        action.angular.z = -0.3; // Slight right turn
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on the left, turning slightly right.");
      } else if (detections["Right"]) {
        action.linear.x = 0.2;  // Move forward slowly
        action.angular.z = 0.3; // Slight left turn
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on the right, turning slightly left.");
      }
      // Priority 3: Rear detections
      else if (detections["Right_Rear"]) {
        action.linear.x = 0.3; // Move forward
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on the right rear, moving forward.");
      } else if (detections["Left_Rear"]) {
        action.linear.x = 0.3; // Move forward
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle on the left rear, moving forward.");
      } else {
        action.linear.x = 0.5; // Move forward
        RCLCPP_INFO(this->get_logger(), "No obstacles, moving forward.");
      }
    }

    // Publish the action command
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