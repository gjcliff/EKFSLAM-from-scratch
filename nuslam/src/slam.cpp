#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam"), count_(0)
  {
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/green/odom", 10, std::bind(&Slam::odometry_callback, this, _1));

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "slam_path", 10);

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, _1));

    // initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    n_ = 3;
    correction_ = arma::zeros(9,1);
    sigma_ = arma::zeros(9, 9);
    Qbar_ = arma::zeros(9, 9);
    A_ = arma::zeros(9, 9);
    H_ = arma::zeros(9, 9);
    xi_ = arma::zeros(9, 1);
    map_ = arma::zeros(6, 1);
    Qfactor_ = 10.0;
    Rfactor_ = 100.0;
    R_ = arma::eye(2, 2) * Rfactor_;
    
    initialize_xi();
    initialize_Q();
    initialize_sigma();

    noise_generator_ = std::normal_distribution<>(0, Rfactor_);
    path_.header.frame_id = "nusim/world";
  }

private:
  geometry_msgs::msg::PoseStamped construct_path_msg()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = xi_(1);
    pose.pose.position.y = xi_(2);

    tf2::Quaternion q_tf2;
    q_tf2.setRPY(0, 0, xi_(0));
    pose.pose.orientation = tf2::toMsg(q_tf2);

    return pose;
  }

  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }

  void initialize_xi()
  {
    xi_ = arma::join_vert(arma::zeros(3, 1), map_);
  }

  void initialize_Q()
  {
    Qbar_ = arma::eye(3, 3);
    Qbar_ *= Qfactor_;
    Qbar_ = arma::join_vert(Qbar_, arma::zeros(2 * n_, 3));
    Qbar_ = arma::join_horiz(Qbar_, arma::zeros(3 + 2 * n_, 2 * n_));
  }

  void initialize_sigma()
  {
    arma::mat sigma_q = arma::zeros(3, 3);
    arma::mat sigma_m = arma::ones(2 * n_, 2 * n_) * 1000000.0;
    arma::mat sigma_qz = arma::join_vert(sigma_q, arma::zeros(2 * n_, 3));
    arma::mat sigma_mz = arma::join_vert(arma::zeros(3, 2 * n_), sigma_m);
    sigma_ = arma::join_horiz(sigma_qz, sigma_mz);
  }

  arma::mat calculate_A(turtlelib::Twist2D Vb)
  {
    // TODO: Clean this up
    arma::mat A;
    if (Vb.omega < 1e-3) {
      arma::mat tmp = {{0, 0, 0},
        {-Vb.x * std::sin(xi_(0)), 0, 0},
        {Vb.x * std::cos(xi_(0)), 0, 0}};
      tmp = arma::join_vert(tmp, arma::zeros(2 * n_, 3));
      A = arma::join_horiz(tmp, arma::zeros(3 + 2 * n_, 2 * n_));
      A = arma::eye(3 + 2 * n_, 3 + 2 * n_) + A;
    } else {
      arma::mat tmp = {{0, 0, 0},
        {-Vb.x / Vb.omega * std::cos(xi_(0)) + Vb.x / Vb.omega *
          std::cos(
            turtlelib::normalize_angle(
              xi_(0) + Vb.omega)), 0, 0},
        {-Vb.x / Vb.omega * std::sin(xi_(0)) + Vb.x / Vb.omega *
          std::sin(
            turtlelib::normalize_angle(
              xi_(0) + Vb.omega)), 0, 0}};
      tmp = arma::join_vert(tmp, arma::zeros(2 * n_, 3));
      A = arma::join_horiz(tmp, arma::zeros(3 + 2 * n_, 2 * n_));
      A = arma::eye(3 + 2 * n_, 3 + 2 * n_) + A;
    }

    return A;
  }

  arma::mat calc_Hj(double mx, double my, int j)
  {
    // TODO: Clean this up
    // xi_minus_?
    double delta_x = mx - xi_(1);
    double delta_y = my - xi_(2);
    double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);

    arma::mat block1{{0, -delta_x / std::sqrt(d), -delta_y / std::sqrt(d)},
      {-1, delta_y / d, -delta_x / d}};
    arma::mat block2 = arma::join_vert(arma::zeros(1, 2 * (j)), arma::zeros(1, 2 * (j)));
    arma::mat block3{{delta_x / std::sqrt(d), delta_y / std::sqrt(d)},
      {-delta_y / d, delta_x / d}};
    arma::mat block4 =
      arma::join_vert(arma::zeros(1, 2 * n_ - 2 * (j + 1)), arma::zeros(1, 2 * n_ - 2 * (j + 1)));

    arma::mat block12 = arma::join_horiz(block1, block2);
    arma::mat block34 = arma::join_horiz(block3, block4);
    arma::mat H = arma::join_horiz(block12, block34);

    return H;
  }

  void initialize_map(const visualization_msgs::msg::MarkerArray & msg)
  {
    for (int j = 0; j < static_cast<int>(msg.markers.size()); j++) {
      turtlelib::Point2D m = map_to_robot_(turtlelib::Point2D{msg.markers.at(j).pose.position.x,
        msg.markers.at(j).pose.position.y});
      map_ids_.push_back(msg.markers.at(j).id);
      map_(j*2) = m.x;
      map_(j*2+1) = m.y;
    }
  } 

  /// @brief When a new obstacle estimate is received, update the current, previous,
  /// and delta configuration variables. Also update the map, and update the H matrix.
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    builtin_interfaces::msg::Time time = get_clock()->now();

    if (count_ == 0) {
      initialize_map(msg);
    }

    // TODO: Clean this up
    for (int j = 0; j < static_cast<int>(msg.markers.size()); j++) {
      // make some useful variables
      // this is the location of the marker in the robot's frame
      turtlelib::Point2D m = map_to_robot_(turtlelib::Point2D{msg.markers.at(j).pose.position.x,
        msg.markers.at(j).pose.position.y});

      // update the measurement model
      arma::mat Hj = calc_Hj(m.x, m.y, j);

      // perform the SLAM update

      // calculate the Kalman gain
      arma::mat K = sigma_ * Hj.t() * arma::inv(Hj * sigma_ * Hj.t() + R_);

      // calculate the position of the robot based on current measurements
      arma::colvec z(2, arma::fill::zeros);
      z(0) = std::sqrt(std::pow(m.x - xi_(1),2) + std::pow(m.y - xi_(2), 2));
      z(1) = turtlelib::normalize_angle(std::atan2(m.y - xi_(2), m.x - xi_(1)) - xi_(0));

      // calculate the position of the robot based on previous measurements
      arma::colvec z_hat(2, arma::fill::zeros);
      z_hat(0) = std::sqrt(std::pow(map_(j*2) - xi_(1), 2) + std::pow(map_(j*2+1) - xi_(2), 2));
      z_hat(1) = turtlelib::normalize_angle(std::atan2(map_(j*2+1) - xi_(2), map_(j*2) - xi_(1)) - xi_(0));

      // calculate the correction to the robot's position
      correction_ = K * (z - z_hat);
      turtlelib::Transform2D map_to_odom_new{{correction_(1), correction_(2)}, correction_(0)};
      RCLCPP_INFO_STREAM(get_logger(), "j: " << j);
      RCLCPP_INFO_STREAM(get_logger(), "j: " << j);
      RCLCPP_INFO_STREAM(get_logger(), "obstacle id: " << msg.markers.at(j).id);
      RCLCPP_INFO_STREAM(get_logger(), "map_to_odom_new: " << map_to_odom_new);
      RCLCPP_INFO_STREAM(get_logger(), "z: \n" << z.t());
      RCLCPP_INFO_STREAM(get_logger(), "z_hat: \n" << z_hat.t());
      RCLCPP_INFO_STREAM(get_logger(), "std::atan2(m.y - xi_(2), m.x - xi_(1)): " << std::atan2(m.y - xi_(2), m.x - xi_(1)));
      RCLCPP_INFO_STREAM(get_logger(), "turtlelib::normalize_angle(std::atan2(m.y - xi_(2), m.x - xi_(1)) - xi_(0)): " << turtlelib::normalize_angle(std::atan2(m.y - xi_(2), m.x - xi_(1)) - xi_(0)));
      RCLCPP_INFO_STREAM(get_logger(), "std::atan2(map_(j*2+1) - xi_(2), map_(j*2) - xi_(1)): " << std::atan2(map_(j*2+1) - xi_(2), map_(j*2) - xi_(1)));
      RCLCPP_INFO_STREAM(get_logger(), "turtlelib::normalize_angle(std::atan2(map_(j*2+1) - xi_(2), map_(j*2) - xi_(1)) - xi_(0)): " << turtlelib::normalize_angle(std::atan2(map_(j*2+1) - xi_(2), map_(j*2) - xi_(1)) - xi_(0)));
      RCLCPP_INFO_STREAM(get_logger(), "correction: \n" << correction_.t());

      RCLCPP_INFO_STREAM(get_logger(), "xi_: \n" << xi_.t());
      RCLCPP_INFO_STREAM(get_logger(), "m: " << m.x << ", " << m.y);

      map_to_odom_ = map_to_odom_new * map_to_odom_;

      // don't think this matters at all, but ok!
      xi_ = xi_ + correction_;

      RCLCPP_INFO_STREAM(get_logger(), "xi_ after: \n" << xi_.t());

      // compute the posterior covariance
      sigma_ = (arma::eye(9, 9) - K * Hj) * sigma_;

      // why is the map drifting?
      xi_.rows(3,8) = map_;

      // correct the position of the robot in the map frame by updating
      // the transform from map to odom
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = time;
      t.header.frame_id = "map";
      t.child_frame_id = "green/odom";

      tf2::Quaternion q_correct;
      q_correct.setRPY(0, 0, map_to_odom_.rotation());

      t.transform.translation.x = map_to_odom_.translation().x;
      t.transform.translation.y = map_to_odom_.translation().y;
      t.transform.rotation = tf2::toMsg(q_correct);

      tf_broadcaster_->sendTransform(t);

      path_.poses.push_back(construct_path_msg());
      path_.header.stamp = time;
      path_publisher_->publish(path_);

      count_++;
    }

    RCLCPP_INFO_STREAM(get_logger(), "\n\n\n\n");

  }

  void odometry_callback(const nav_msgs::msg::Odometry & msg)
  {
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Vb_ = turtlelib::Twist2D{msg.twist.twist.linear.x,
                             msg.twist.twist.linear.y,
                             msg.twist.twist.angular.z};

    odom_to_robot_ = turtlelib::Transform2D{{msg.pose.pose.position.x, msg.pose.pose.position.y}, yaw};
    map_to_robot_ = map_to_odom_ * odom_to_robot_;
    xi_(0) = map_to_robot_.rotation();
    xi_(1) = map_to_robot_.translation().x;
    xi_(2) = map_to_robot_.translation().y;

    A_ = calculate_A(Vb_);
    sigma_ = A_ * sigma_ * A_.t() + Qbar_;

  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  // transform broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;

  arma::mat correction_;

  arma::mat map_;
  arma::mat xi_;
  arma::mat sigma_;
  arma::mat Qbar_;
  arma::mat A_;
  arma::mat H_;
  arma::mat R_;
  turtlelib::Twist2D Vb_;
  turtlelib::Twist2D Vb_minus_;
  turtlelib::Transform2D map_to_robot_; // map is always at (0,0) for us
  turtlelib::Transform2D odom_to_robot_;
  turtlelib::Transform2D map_to_odom_;
  double Qfactor_;
  double Rfactor_;

  double n_;
  vector<double> map_ids_;

  double count_;
  std::normal_distribution<> noise_generator_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
