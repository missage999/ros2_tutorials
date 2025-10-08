#include "rclcpp/rclcpp.hpp"
#include "srv_param/srv/set_max_velocity.hpp"

class VelocityManager : public rclcpp::Node
{
public:
  VelocityManager() : Node("velocity_manager")
  {
    // 声明参数：max_velocity，默认1.0 m/s
    this->declare_parameter("max_velocity", 1.0);

    // 创建服务
    service_ = this->create_service<srv_param::srv::SetMaxVelocity>(
      "/set_max_velocity",
      std::bind(&VelocityManager::handle_set_max_velocity, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "VelocityManager started. Max velocity: %.2f m/s",
                this->get_parameter("max_velocity").as_double());
  }

private:
  void handle_set_max_velocity(
    const std::shared_ptr<srv_param::srv::SetMaxVelocity::Request> request,
    std::shared_ptr<srv_param::srv::SetMaxVelocity::Response> response)
  {
    double new_vel = request->max_vel;

    if (new_vel > 0.0 && new_vel <= 5.0) {
      this->set_parameter(rclcpp::Parameter("max_velocity", new_vel));
      response->success = true;
      response->message = "Max velocity updated to " + std::to_string(new_vel) + " m/s";
      RCLCPP_INFO(this->get_logger(), "Max velocity set to: %.2f", new_vel);
    } else {
      response->success = false;
      response->message = "Invalid velocity: must be in (0, 5] m/s";
      RCLCPP_WARN(this->get_logger(), "Rejected velocity: %.2f", new_vel);
    }
  }

  rclcpp::Service<srv_param::srv::SetMaxVelocity>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityManager>());
  rclcpp::shutdown();
  return 0;
}