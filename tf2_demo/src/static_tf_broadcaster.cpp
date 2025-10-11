// #include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/static_transform_broadcaster.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// using namespace std::chrono_literals;

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("static_tf_broadcaster");

//   tf2_ros::StaticTransformBroadcaster broadcaster(node);

//   geometry_msgs::msg::TransformStamped static_transform;
//   static_transform.header.stamp = node->now();
//   static_transform.header.frame_id = "base_link";    // 父坐标系
//   static_transform.child_frame_id = "laser";         // 子坐标系

//   // 平移：x=0.2m（前方），y=0, z=0.1m（略高）
//   static_transform.transform.translation.x = 0.2;
//   static_transform.transform.translation.y = 0.0;
//   static_transform.transform.translation.z = 0.1;

//   // 无旋转：单位四元数
//   static_transform.transform.rotation.x = 0.0;
//   static_transform.transform.rotation.y = 0.0;
//   static_transform.transform.rotation.z = 0.0;
//   static_transform.transform.rotation.w = 1.0;

//   broadcaster.sendTransform(static_transform);

//   RCLCPP_INFO(node->get_logger(), "Published static transform: base_link -> laser");

//   // 静态TF只需发布一次，但保持节点运行以便调试
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticTfBroadcaster : public rclcpp::Node
{
public:
  StaticTfBroadcaster()
  : Node("static_tf_broadcaster")
  {
    // 创建静态 TF 广播器
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 构造 TransformStamped 消息
    geometry_msgs::msg::TransformStamped static_transform;

    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "base_link";   // 父坐标系
    static_transform.child_frame_id = "laser";        // 子坐标系

    // 平移：前方 0.2m，高度 0.1m
    static_transform.transform.translation.x = 0.2;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.1;

    // 无旋转：单位四元数
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    // 发布静态 TF（只需一次）
    static_broadcaster_->sendTransform(static_transform);

    RCLCPP_INFO(this->get_logger(), "Published static transform: base_link -> laser");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StaticTfBroadcaster>();

  // 保持节点运行（静态 TF 只需发布一次，但节点常驻便于调试）
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}