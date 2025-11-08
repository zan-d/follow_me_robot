#include "action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class RobotMovingPublisher : public rclcpp::Node
{
public:
  RobotMovingPublisher()
  : Node("robot_moving_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("robot_moving", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds((int)(1.0f/action_node_hz * 1000)),  // action_node_hz Hz
      std::bind(&RobotMovingPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "RobotMovingPublisher started, publishing False on robot_moving");
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Bool msg;
    msg.data = false;
    publisher_->publish(msg);
    //RCLCPP_INFO(this->get_logger(), "Published: False");
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMovingPublisher>());
  rclcpp::shutdown();
  return 0;
}
