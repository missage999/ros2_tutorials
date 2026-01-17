#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
public:
  TFListener() : Node("tf2_listener")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(1s, std::bind(&TFListener::lookup_transform, this));
  }

private:
  void lookup_transform()
  {
    try {
      // 查询 laser 在 odom 下的位置
      auto transform = tf_buffer_->lookupTransform(
        "odom",      // 目标坐标系
        "laser",     // 源坐标系
        tf2::TimePointZero  // 最新可用
      );

      RCLCPP_INFO(this->get_logger(),
        "laser in odom: x=%.2f, y=%.2f, z=%.2f",
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFListener>());
  rclcpp::shutdown();
  return 0;
}