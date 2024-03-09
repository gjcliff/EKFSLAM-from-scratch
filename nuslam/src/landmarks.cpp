#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Landmarks : public rclcpp::Node
{
  public:
    Landmarks()
    : Node("landmarks"), count_(0)
    {
      scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10, std::bind(&Landmarks::scan_callback, this, std::placeholders::_1));
      timer_ = create_wall_timer(
      500ms, std::bind(&Landmarks::timer_callback, this));
    }

  private:
    void circular_regression()
    {
      double x_hat = 0;
      double y_hat = 0;
      // find the centroid of the laser scan cluseters
      for (int i = 0; i < 69; i++) {
      }
    }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr & msg) const
    {
      
    }
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
