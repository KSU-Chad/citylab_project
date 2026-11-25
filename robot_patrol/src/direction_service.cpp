#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service") {

    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::direction_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // RCLCPP_INFO(this->get_logger(), "Patrol Node Ready...");
  }

private:
  void direction_callback(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {
    const auto &scan = request->laser_data;
    int n = static_cast<int>(scan.ranges.size());
    float inc_deg = scan.angle_increment * 180.0f / M_PI;
    float angle_min_deg = scan.angle_min * 180.0f / M_PI;

    // convert an angle (deg, 0 = straight ahead) to an index in ranges[]
    auto angle_to_index = [&](float target_deg) {
      float offset = target_deg - angle_min_deg;
      while (offset < 0)
        offset += 360.0f;
      while (offset >= 360.0f)
        offset -= 360.0f;
      return static_cast<int>(offset / inc_deg) % n;
    };

    // add up all valid distances between two angles
    auto sum_sector = [&](float start_deg, float end_deg) {
      int start_idx = angle_to_index(start_deg);
      int end_idx = angle_to_index(end_deg);
      float total = 0.0f;

      auto add_ray = [&](int i) {
        float r = scan.ranges[i];
        if (std::isinf(r) || std::isnan(r) || r < scan.range_min ||
            r > scan.range_max) {
          return;
        }
        total += r;
      };

      if (start_idx <= end_idx) {
        for (int i = start_idx; i <= end_idx; ++i)
          add_ray(i);
      } else { // sector wraps around the end of the array
        for (int i = start_idx; i < n; ++i)
          add_ray(i);
        for (int i = 0; i <= end_idx; ++i)
          add_ray(i);
      }
      return total;
    };

    // smallest valid distance between two angles
    auto min_sector = [&](float start_deg, float end_deg) {
      int start_idx = angle_to_index(start_deg);
      int end_idx = angle_to_index(end_deg);
      float min_val = std::numeric_limits<float>::infinity();

      auto check_ray = [&](int i) {
        float r = scan.ranges[i];
        if (std::isinf(r) || std::isnan(r) || r < scan.range_min ||
            r > scan.range_max) {
          return;
        }
        if (r < min_val)
          min_val = r;
      };

      if (start_idx <= end_idx) {
        for (int i = start_idx; i <= end_idx; ++i)
          check_ray(i);
      } else { // sector wraps around the end of the array
        for (int i = start_idx; i < n; ++i)
          check_ray(i);
        for (int i = 0; i <= end_idx; ++i)
          check_ray(i);
      }
      return min_val;
    };

    float total_dist_sec_right = sum_sector(-90.0f, -30.0f);
    float total_dist_sec_front = sum_sector(-30.0f, 30.0f);
    float total_dist_sec_left = sum_sector(30.0f, 90.0f);

    float min_dist_front = min_sector(-30.0f, 30.0f);

    if (min_dist_front > 0.35f) {
      response->direction = "forward";
    } else if (total_dist_sec_left > total_dist_sec_right) {
      response->direction = "left";
    } else {
      response->direction = "right";
    }
  }
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}