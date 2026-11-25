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
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {}
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}