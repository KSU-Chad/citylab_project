#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service") {
    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&TestService::laserscan_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        1s, std::bind(&TestService::timer_callback, this));
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = *msg;
  }

  void timer_callback() {
    // nothing to send until a scan has arrived and the server is up
    if (latest_scan_.ranges.empty() || !client_->service_is_ready()) {
      return;
    }

    auto request =
        std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = latest_scan_;

    client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                   future) {
          RCLCPP_INFO(this->get_logger(), "Direction: %s",
                      future.get()->direction.c_str());
        });
  }

  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan latest_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}