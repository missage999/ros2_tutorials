#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>  // 对应 Python 的 turtlesim.msg.Pose

class TurtleSubscriber : public rclcpp::Node
{
public:
  TurtleSubscriber()
  : Node("turtle_subscriber")
  {
    // 创建 Subscriber：话题 /turtle1/pose，队列 10
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose",
      10,
      std::bind(&TurtleSubscriber::listener_callback, this, std::placeholders::_1));
  }

private:
  // 回调函数，参数是 Pose 消息的智能指针
  void listener_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received pose: x=%.2f, y=%.2f, theta=%.2f",
                msg->x, msg->y, msg->theta);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                                // 初始化 ROS 2
  rclcpp::spin(std::make_shared<TurtleSubscriber>());     // 进入事件循环
  rclcpp::shutdown();                                      // 清理
  return 0;
}