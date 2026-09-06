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
    for (const std::string name :
         {"Front_Left", "Front_Right", "Left", "Right"}) {
      min_distances_[name] = std::numeric_limits<float>::infinity();
      max_distances_[name] = 0.0f;
    }

    // Subscriber to LaserScan
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    // Publisher for movement commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol Node Ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // configure lidar sensor details
    int n = static_cast<int>(msg->ranges.size());
    float inc_deg = msg->angle_increment * 180.0f / M_PI;
    float angle_min_deg = msg->angle_min * 180.0f / M_PI;

    // calculate lidar sectors based on sensor parameters
    auto angle_to_index = [&](float target_deg) {
      float offset_from_min = target_deg - angle_min_deg;
      while (offset_from_min < 0)
        offset_from_min += 360.0f;
      while (offset_from_min >= 360.0f)
        offset_from_min -= 360.0f;
      return static_cast<int>(offset_from_min / inc_deg) % n;
    };

    float narrow_half = narrow_range_deg_ / 2.0f; // 20
    float scan_half = scan_range_deg_ / 2.0f;     // 90

    // calculate sector divisions based on lidar data
    std::map<std::string, std::pair<int, int>> sectors = {
        {"Front_Left", {angle_to_index(0.0f), angle_to_index(narrow_half)}},
        {"Front_Right", {angle_to_index(-narrow_half), angle_to_index(0.0f)}},
        {"Left", {angle_to_index(narrow_half), angle_to_index(scan_half)}},
        {"Right", {angle_to_index(-scan_half), angle_to_index(-narrow_half)}}};

    for (const auto &sector : sectors) {
      min_distances_[sector.first] = std::numeric_limits<float>::infinity();
      max_distances_[sector.first] = 0.0f;
    }

    // Find the minimum/maximum distance in each sector
    for (const auto &sector : sectors) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;
      float min_val = std::numeric_limits<float>::infinity();
      float max_val = 0.0f;

      auto check_ray = [&](int i) {
        float range = msg->ranges[i];
        if (std::isinf(range) || std::isnan(range) || range < msg->range_min ||
            range > msg->range_max) {
          return;
        }
        if (range < min_val) {
          min_val = range;
        }
        if (range > max_val) {
          max_val = range;
        }
      };

      if (start_idx <= end_idx) {
        for (int i = start_idx; i <= end_idx; ++i)
          check_ray(i);
      } else {
        for (int i = start_idx; i < n; ++i)
          check_ray(i);
        for (int i = 0; i <= end_idx; ++i)
          check_ray(i);
      }

      min_distances_[sector.first] = min_val;
      max_distances_[sector.first] = max_val;
    }
  }

  void control_loop() {
    auto action = geometry_msgs::msg::Twist();
    // threshold for obstacle detection
    float obstacle_threshold = 0.35; // meters

    // Narrow front check
    float min_narrow =
        std::min(min_distances_["Front_Left"], min_distances_["Front_Right"]);

    // check sides
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
  // sector width constants
  float narrow_range_deg_ = 40.0f;
  float scan_range_deg_ = 180.0f;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}