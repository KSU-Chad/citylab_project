#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {

    // Subscriber to LaserScan
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    // Publisher for movement commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol Node Ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = *msg;
  }

  void control_loop() {
    auto action = geometry_msgs::msg::Twist();

    if (!latest_scan_.ranges.empty() && client_->service_is_ready()) {
      auto request =
          std::make_shared<robot_patrol::srv::GetDirection::Request>();
      request->laser_data = latest_scan_;
      client_->async_send_request(
          request,
          [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                     future) { direction_ = future.get()->direction; });
    }

    if (direction_ == "forward") {
      action.linear.x = 0.1;
      action.angular.z = 0.0;
    } else if (direction_ == "left") {
      action.linear.x = 0.1;
      action.angular.z = 0.5;
    } else if (direction_ == "right") {
      action.linear.x = 0.1;
      action.angular.z = -0.5;
    }
    
    publisher_->publish(action);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan latest_scan_;
  std::string direction_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
