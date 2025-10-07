#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class TurtlePublisher : public rclcpp::Node
{
public:
  TurtlePublisher()
  : Node("turtle_publisher")
  {
    // 创建 Publisher：发布到 "/turtle1/cmd_vel"，队列大小10
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    
    // 创建定时器：每500ms调用一次回调函数
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TurtlePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 2.0;   // 前进速度
    message.angular.z = 1.0;  // 旋转速度 → 画圆！

    RCLCPP_INFO(this->get_logger(), "Publishing: linear=%.2f, angular=%.2f", 
                message.linear.x, message.angular.z);
    
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                    // 初始化 ROS 2
  rclcpp::spin(std::make_shared<TurtlePublisher>()); // 进入事件循环
  rclcpp::shutdown();                          // 关闭
  return 0;
}