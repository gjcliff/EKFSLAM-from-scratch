#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class TurtleSimulation : public rclcpp::Node
{
public:
  TurtleSimulation()
  : Node("nusim")
  {
    // declare parameters
    declare_parameter("rate", 5);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_length_x", 1.0);
    declare_parameter("arena_length_y", 1.0);
    declare_parameter("obstacles/x", std::vector<double>({0.0, 0.0}));
    declare_parameter("obstacles/y", std::vector<double>({0.0, 0.0}));
    declare_parameter("obstacles/height", 0.25);
    declare_parameter("obstacles/r", 0.25);

    declare_parameter("wheel_radius", 0.1);
    declare_parameter("track_width", 1.0);
    declare_parameter("motor_cmd_max", 100);
    declare_parameter("motor_cmd_per_rad_sec", 0.01);
    declare_parameter("encoder_ticks_per_rad", 500.0);
    declare_parameter("collision_radius", 0.2);

    // set parameters
    rate_ = get_parameter("rate").as_int();
    x_ = get_parameter("x0").as_double();
    y_ = get_parameter("y0").as_double();
    theta_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_length_x").as_double();
    arena_y_length_ = get_parameter("arena_length_y").as_double();
    obstacles_x = get_parameter("obstacles/x").as_double_array();
    obstacles_y = get_parameter("obstacles/y").as_double_array();
    obstacle_height_ = get_parameter("obstacles/height").as_double();
    obstacle_radius_ = get_parameter("obstacles/r").as_double();

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();

    turtlelib::RobotDimensions rd{0.0, track_width_ / 2, wheel_radius_};
    turtlebot_.set_robot_dimensions(rd);

    // set miscellaneous variables
    og_x_ = x_;
    og_y_ = y_;
    og_theta_ = theta_;

    // check whether or not the obstacle arrays are the same size
    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_ERROR(get_logger(), "obstacles_x and obstacles_y are not the same size");
      rclcpp::shutdown();
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

    // declare publishers
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", qos);
    walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);
    obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", qos);
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);

    // declare subscribers
    wheel_commands_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "/wheel_cmd", 10, std::bind(&TurtleSimulation::wheel_commands_callback, this, _1));

    // declare static transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // declare services
    reset_service_ =
      create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&TurtleSimulation::reset_callback, this, _1, _2));
    teleport_service_ =
      create_service<nusim::srv::Teleport>(
      "teleport",
      std::bind(&TurtleSimulation::teleport_callback, this, _1, _2));

    // initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_->sendTransform(construct_transform_msg(x_, y_, theta_));
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(rate_),
      std::bind(&TurtleSimulation::timer_callback, this));

    visualization_msgs::msg::MarkerArray wall_array = construct_wall_array();
    walls_publisher_->publish(wall_array);

  }

private:
  void wheel_commands_callback(const nuturtlebot_msgs::msg::WheelCommands msg)
  {
    double left_wheel_velocity = msg.left_velocity * motor_cmd_per_rad_sec_;
    double right_wheel_velocity = msg.right_velocity * motor_cmd_per_rad_sec_;

    turtlelib::Twist2D Vb = turtlebot_.FK(left_wheel_velocity, right_wheel_velocity);
    vector<turtlelib::Configuration> qv = turtlebot_.update_configuration(Vb);

    x_ = qv.at(0).x;
    y_ = qv.at(0).y;
    theta_ = qv.at(0).theta;

    vector<double> wheel_pos_rad = turtlebot_.IK(Vb);
    left_encoder_ticks_ += static_cast<int>(wheel_pos_rad.at(0) * encoder_ticks_per_rad_);
    right_encoder_ticks_ += static_cast<int>(wheel_pos_rad.at(1) * encoder_ticks_per_rad_);
  }

  visualization_msgs::msg::MarkerArray construct_obstacle_array()
  {
    visualization_msgs::msg::MarkerArray arr;
    for (int i = 0; i < (int)obstacles_x.size(); i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = get_clock()->now();
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacles_x[i];
      marker.pose.position.y = obstacles_y[i];
      marker.pose.position.z = wall_height_ / 2;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_radius_;
      marker.scale.y = obstacle_radius_;
      marker.scale.z = obstacle_height_;
      marker.color.a = 1.0;
      marker.color.r = 1.0;

      arr.markers.push_back(marker);
    }

    return arr;
  }

  visualization_msgs::msg::Marker construct_marker(
    double pos_x, double pos_y, double scale_x,
    double scale_y, int id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pos_x;
    marker.pose.position.y = pos_y;
    marker.pose.position.z = wall_height_ / 2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = wall_height_;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    return marker;
  }

  visualization_msgs::msg::MarkerArray construct_wall_array()
  {
    visualization_msgs::msg::MarkerArray wall_array;

    double x_length = (arena_x_length_ + wall_thickness_);
    double y_length = (arena_y_length_ + wall_thickness_);

    wall_array.markers.push_back(
      construct_marker(
        arena_x_length_ / 2, 0.0, wall_thickness_,
        x_length, 0));
    wall_array.markers.push_back(
      construct_marker(
        0.0, arena_y_length_ / 2, y_length,
        wall_thickness_, 1));
    wall_array.markers.push_back(
      construct_marker(
        -arena_x_length_ / 2, 0.0, wall_thickness_,
        x_length, 2));
    wall_array.markers.push_back(
      construct_marker(
        0.0, -arena_y_length_ / 2, y_length,
        wall_thickness_, 3));

    return wall_array;
  }

  geometry_msgs::msg::TransformStamped construct_transform_msg(double x, double y, double theta)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q_tf2;
    q_tf2.setRPY(0, 0, theta);

    geometry_msgs::msg::Quaternion q = tf2::toMsg(q_tf2);

    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;
    t.transform.rotation.w = q.w;

    return t;
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    geometry_msgs::msg::TransformStamped t = construct_transform_msg(
      request->x, request->y,
      request->theta);

    tf_broadcaster_->sendTransform(t);
    (void) response;

    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    current_timestep_ = 0;
    x_ = og_x_;
    y_ = og_y_;
    theta_ = og_theta_;
  }

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t = construct_transform_msg(x_, y_, theta_);
    tf_broadcaster_->sendTransform(t);

    // keep track of the current timestep
    current_timestep_ += 1;
    auto timestep_message = std_msgs::msg::UInt64();
    timestep_message.data = current_timestep_;
    timestep_publisher_->publish(timestep_message);

    visualization_msgs::msg::MarkerArray wall_array = construct_wall_array();
    walls_publisher_->publish(wall_array);

    visualization_msgs::msg::MarkerArray obstacle_array = construct_obstacle_array();
    obstacles_publisher_->publish(obstacle_array);


    nuturtlebot_msgs::msg::SensorData sensor_data_msg;
    sensor_data_msg.left_encoder = left_encoder_ticks_;
    sensor_data_msg.right_encoder = right_encoder_ticks_;
    sensor_data_msg.stamp = get_clock()->now();

    sensor_data_publisher_->publish(sensor_data_msg);
    left_encoder_ticks_ = 0.0;
    right_encoder_ticks_ = 0.0;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  // publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  // subscribers
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_commands_subscriber_;
  // transform broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;

  turtlelib::DiffDrive turtlebot_;

  int rate_ = 0;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double og_x_;
  double og_y_;
  double og_theta_;
  double arena_x_length_;
  double arena_y_length_;
  double wall_height_ = 0.25;
  double wall_thickness_ = 0.05;
  double wheel_radius_;
  double track_width_;
  int motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  double left_encoder_ticks_;
  double right_encoder_ticks_;
  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacle_radius_;
  double obstacle_height_ = 0.25;
  unsigned int current_timestep_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleSimulation>());
  rclcpp::shutdown();
  return 0;
}
