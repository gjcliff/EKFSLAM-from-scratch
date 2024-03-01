#include <chrono>
#include <string>
#include <turtlelib/se2d.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <armadillo>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Slam : public rclcpp::Node
{
  public:
    Slam()
    : Node("slam"), count_(0)
    {
      odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "green/odom", 10, std::bind(&Slam::odometry_callback, this, _1));

      fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, _1));

      // declare static transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      n_ = 3;
      sigma_ = arma::zeros(9,9);
      Qbar_ = arma::zeros(9,9);
      A_ = arma::zeros(9,9);
      H_ = arma::zeros(9,9);
      xi_ = arma::zeros(9,1);
      map_ = arma::zeros(6,1);
      Qfactor_ = 5.0;
      Rfactor_ = 5.0;
      R_ = arma::eye(2,2) * Rfactor_;

      noise_generator_ = std::normal_distribution<>(0, Rfactor_);
    }

  private:
    std::mt19937 & get_random()
    {
      // static variables inside a function are created once and persist for the remainder of the program
      static std::random_device rd{}; 
      static std::mt19937 mt{rd()};
      // we return a reference to the pseudo-random number genrator object. This is always the
      // same object every time get_random is called
      return mt;
    }
    void initialize_map(const visualization_msgs::msg::MarkerArray & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Initializing map");
      map_.zeros(msg.markers.size() * 2, 1);
      n_ = msg.markers.size();
      for (int i = 0; i < static_cast<int>(msg.markers.size()); i++) {
        map_ids_.push_back(msg.markers.at(i).id);
      }
    }

    void initialize_xi()
    {
      RCLCPP_INFO(this->get_logger(), "Initializing xi");
      xi_ = arma::join_vert(arma::zeros(3,1), map_);
      RCLCPP_INFO_STREAM(this->get_logger(), "xi: " << xi_);
    }

    void initialize_Q()
    {
      RCLCPP_INFO(this->get_logger(), "Initializing Q");
      Qbar_ = arma::eye(3,3);
      Qbar_ *= Qfactor_;
      Qbar_ = arma::join_vert(Qbar_, arma::zeros(2*n_, 3));
      Qbar_ = arma::join_horiz(Qbar_, arma::zeros(3 + 2*n_, 2*n_));
    }

    void initialize_sigma()
    {
      RCLCPP_INFO(this->get_logger(), "Initializing sigma");
      arma::mat sigma_q = arma::zeros(3,3);
      arma::mat sigma_m = arma::ones(2*n_, 2*n_) * 1000000.0;
      arma::mat sigma_qz = arma::join_vert(sigma_q, arma::zeros(2*n_,3));
      arma::mat sigma_mz = arma::join_vert(arma::zeros(3, 2*n_), sigma_m);

      sigma_ = arma::join_horiz(sigma_qz, sigma_mz);
    }

    arma::mat calculate_A(turtlelib::Twist2D Vb)
    {
      // TODO: Clean this up
      arma::mat A;
      if (Vb.omega < 1e-3) {
        arma::mat tmp = {{0,0,0},
                         {-Vb.x * std::sin(xi_(2)),0,0},
                         {Vb.x * std::cos(xi_(2)),0,0}};
        tmp = arma::join_vert(tmp, arma::zeros(2*n_, 3));
        A = arma::join_horiz(tmp,arma::zeros(3+2*n_,2*n_));
        A = arma::eye(3 + 2*n_,3 + 2*n_) + A;
      } else {
        arma::mat tmp = {{0,0,0},
                         {-Vb.x / Vb.omega * std::cos(xi_(2)) + Vb.x / Vb.omega * std::cos(turtlelib::normalize_angle(xi_(2) + Vb.omega)),0,0},
                         {-Vb.x / Vb.omega * std::sin(xi_(2)) + Vb.x / Vb.omega * std::sin(turtlelib::normalize_angle(xi_(2) + Vb.omega)),0,0}};
        tmp = arma::join_vert(tmp, arma::zeros(2 * n_, 3));
        A = arma::join_horiz(tmp,arma::zeros(3 + 2 * n_, 2 * n_));
        A = arma::eye(3+2*n_,3+2*n_) + A;
      }

      return A;
    }

    arma::mat calc_Hj(double mx, double my, int j)
    {
      // TODO: Clean this up
      double delta_x = mx - xi_(0);
      double delta_y = my - xi_(1);
      double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);

      arma::mat block1{{0, -delta_x/std::sqrt(d), -delta_y/std::sqrt(d)},
                          {-1, delta_y/d, -delta_x/d}};
      arma::mat block2 = arma::join_vert(arma::zeros(1,2*(j)),arma::zeros(1,2*(j)));
      arma::mat block3{{delta_x/std::sqrt(d), delta_y/std::sqrt(d)},
                          {-delta_y/d, delta_x/d}};
      arma::mat block4 = arma::join_vert(arma::zeros(1,2*n_-2*(j+1)),arma::zeros(1,2*n_-2*(j+1)));

      arma::mat block12 = arma::join_horiz(block1, block2);
      arma::mat block34 = arma::join_horiz(block3, block4);
      arma::mat H = arma::join_horiz(block12, block34);

      return H;           
    }

    arma::mat update_state(const turtlelib::Twist2D & Vb)
    {
      if (std::abs(Vb.omega) < 1e-3) {
        return xi_ + arma::join_vert(arma::colvec{0.0,
                                     Vb.x * std::cos(xi_(2)),
                                     Vb.x * std::sin(xi_(2))}, arma::zeros(2*n_));
      } else {
        return xi_ + arma::join_vert(arma::colvec{Vb.omega,
            -Vb.x/Vb.omega * std::sin(xi_(2)) + Vb.x/Vb.omega * std::sin(turtlelib::normalize_angle(xi_(2) + Vb.omega)),
            Vb.x/Vb.omega * std::cos(xi_(2)) - Vb.x/Vb.omega * std::cos(turtlelib::normalize_angle(xi_(2) + Vb.omega))}, arma::zeros(2*n_));
      }
    }


    /// @brief When a new obstacle estimate is received, update the current, previous,
    /// and delta configuration variables. Also update the map, and update the H matrix.
    void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg) 
    {
      // initialize some variables we'll be using

      // TODO: Clean this up

      if (map_.size() == 0 && A_.size() == 0) {
        initialize_map(msg);
        initialize_xi();
        initialize_Q();
        initialize_sigma();
        return;
      } else if (A_.size() == 0){
        return;
      }

      for (int j = 0; j < static_cast<int>(msg.markers.size()); j++) {
        // make some useful variables
        double x = msg.markers.at(j).pose.position.x;
        double y = msg.markers.at(j).pose.position.y;

        // update the measurement model
        arma::mat Hj = calc_Hj(x,y,j);

        // perform the SLAM update
        // TODO: Clean this up
        if (count_ == 0) {
          arma::mat sigma_minus = A_ * sigma_ * A_.t() + Qbar_;
          arma::mat K = sigma_minus * Hj.t() * arma::inv(Hj * sigma_minus * Hj.t() + R_);
          arma::colvec z(2, arma::fill::zeros);
          z(0) = std::sqrt(std::pow(xi_(2 + j*2) - xi_(0), 2) + std::pow(xi_(2 + j*2 + 1) - xi_(1), 2));
          z(1) = turtlelib::normalize_angle(std::atan2(xi_(2+j*2+1) - xi_(1), xi_(2 + j*2) - xi_(0)) - xi_(2));
          arma::colvec z_hat(2, arma::fill::zeros);
          z_hat(0) = std::sqrt(std::pow(x - xi_(0), 2) + std::pow(y - xi_(1), 2));
          z_hat(1) = turtlelib::normalize_angle(std::atan2(y - xi_(1), x - xi_(0)) - xi_(2));

          // print the number of rows and columns in the matrices

          xi_ = xi_ + K * (z - z_hat);
          sigma_ = (arma::eye(9,9) - K * Hj) * sigma_minus;
        } else {
          arma::mat K = sigma_ * Hj.t() * arma::inv(Hj * sigma_ * Hj.t() + R_);
          arma::colvec z(2, arma::fill::zeros);
          z(0) = std::sqrt(std::pow(xi_(2 + j*2) - xi_(0), 2) + std::pow(xi_(2 + j*2 + 1) - xi_(1), 2));
          z(1) = turtlelib::normalize_angle(std::atan2(xi_(2+j*2+1) - xi_(1), xi_(2 + j*2) - xi_(0)) - xi_(2));
          arma::colvec z_hat(2, arma::fill::zeros);
          z_hat(0) = std::sqrt(std::pow(x - xi_(0), 2) + std::pow(y - xi_(1), 2));
          z_hat(1) = turtlelib::normalize_angle(std::atan2(y - xi_(1), x - xi_(0)) - xi_(2));
          xi_ = xi_ + K * (z - z_hat);
          sigma_ = (arma::eye(9,9) - K * Hj) * sigma_;
        }

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = get_clock()->now();
      t.header.frame_id = "map";
      t.child_frame_id = "green/odom";

      t.transform.translation.x = xi_(0);
      t.transform.translation.y = xi_(1);
      t.transform.translation.z = 0.0;

      tf2::Quaternion q_tf2;
      q_tf2.setRPY(0, 0, xi_(2));

      geometry_msgs::msg::Quaternion q = tf2::toMsg(q_tf2);

      t.transform.rotation.x = q.x;
      t.transform.rotation.y = q.y;
      t.transform.rotation.z = q.z;
      t.transform.rotation.w = q.w;

      tf_broadcaster_->sendTransform(t);
      count_++;
      }
    }

    void odometry_callback(const nav_msgs::msg::Odometry & msg)
    {
      if (xi_.is_empty()) {
        return;
      }

      turtlelib::Twist2D Vb = turtlelib::Twist2D{msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z};

      xi_ = update_state(Vb);
      A_ = calculate_A(Vb);
      sigma_ = A_ * sigma_ * A_.t() + Qbar_;

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = get_clock()->now();
      t.header.frame_id = "map";
      t.child_frame_id = "green/odom";

      t.transform.translation.x = xi_(0);
      t.transform.translation.y = xi_(1);

      tf2::Quaternion q_tf2;
      q_tf2.setRPY(0, 0, xi_(2));

      geometry_msgs::msg::Quaternion q = tf2::toMsg(q_tf2);

      t.transform.rotation.x = q.x;
      t.transform.rotation.y = q.y;
      t.transform.rotation.z = q.z;
      t.transform.rotation.w = q.w;

      tf_broadcaster_->sendTransform(t);
    }

    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
    
    // transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    arma::mat map_;
    arma::mat xi_;
    arma::mat sigma_;
    arma::mat Qbar_;
    arma::mat A_;
    arma::mat H_;
    arma::mat R_;
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
