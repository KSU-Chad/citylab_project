#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    // initialize sectors to default values
    for (const auto &sector : sectors_) {
      min_distances_[sector.first] = std::numeric_limits<float>::infinity();
      max_distances_[sector.first] = 0.0f;
    }

    // Subscriber to LaserScan
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    // Publisher for movement commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
     this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol Node Ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // initialize sectors
    for (const auto &sector : sectors_) {
      min_distances_[sector.first] = std::numeric_limits<float>::infinity();
    }
    // Find the minimum distance in each sector
    for (const auto &sector : sectors_) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;

      // Check for invalid data
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
        min_distances_[sector.first] = min_val;
        max_distances_[sector.first] = max_val;
      }
    }
  }

  void control_loop() {
    auto action = geometry_msgs::msg::Twist();
    // Define the threshold for obstacle detection
    float obstacle_threshold = 0.35; // meters

    // Narrow front check
    float min_narrow =
        std::min(min_distances_["Front_Left"], min_distances_["Front_Right"]);

    // Safest direction: side containing the single greatest valid ray
    // as per the instructions
    float left_max =
        std::max(max_distances_["Front_Left"], max_distances_["Left"]);
    float right_max =
        std::max(max_distances_["Front_Right"], max_distances_["Right"]);
    float turn_direction = (left_max >= right_max) ? 1.0f : -1.0f;

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

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Initialize the minimum distances for each sector
  std::map<std::string, float> min_distances_;
  std::map<std::string, float> max_distances_;
  std::map<std::string, std::pair<int, int>> sectors_ = {
      {"Front_Left", {0, 25}},
      {"Left", {26, 112}},
      {"Right", {336, 423}},
      {"Front_Right", {424, 447}}};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}