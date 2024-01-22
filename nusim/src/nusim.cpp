#include<chrono>
#include<functional>
#include<memory>
#include<string>

#include"rclcpp/rclcpp.hpp"
#include"std_msgs/msg/string.hpp"
#include"std_msgs/msg/u_int64.hpp"
#include"std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      this->declare_parameter("rate", 0.5ms);
      publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

      timer_ = this->create_wall_timer(
      0.5ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void reset(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        current_timestep_ = 0;
    }
    
    void timer_callback()
    {
      
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      
      // keep track of the current timestep
      current_timestep_ += 1;
      auto message = std_msgs::msg::UInt64();
      message.data = current_timestep_;
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    size_t count_;
    unsigned int current_timestep_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}