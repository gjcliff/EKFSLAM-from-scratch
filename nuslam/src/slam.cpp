#include <chrono>
#include <string>
#include <turtlelib/se2d.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <armadillo>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

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
        "odom", 10, std::bind(&Slam::odometry_callback, this, _1));

      fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, _1));

      // declare static transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_->sendTransform(construct_transform_msg(x_, y_, theta_));

      timer_ = this->create_wall_timer(
      200ms, std::bind(&Slam::timer_callback, this));

      n_ = 0;
      Qfactor_ = 5.0;
      Rfactor_ = 5.0;
      R_ = arma::eye(2,2) * Rfactor_;
      Qbar_ = initialize_Q();
      sigma_ = initialize_sigma();
      A_ = create_A_matrix();

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
      map_.zeros(msg.markers.size(), 1);
      n_ = msg.markers.size();
      for (int i = 0; i < static_cast<int>(msg.markers.size()); i++) {
        map_ids_.push_back(msg.markers.at(i).id);
      }
    }

    void initialize_xi()
    {
      xi_ = arma::mat({q_.at(0), q_.at(1), q_.at(2)});
      xi_ = arma::join_vert(xi_, map_);
    }

    void add_landmark_to_map(double x, double y, int j, const visualization_msgs::msg::MarkerArray & msg)
    {
      double r = std::sqrt(std::pow(x,2) + std::pow(y,2));
      double phi = std::atan2(y,x);

      map_ids_.push_back(msg.markers.at(j).id);
      n_++;
      map_ = join_vert(map_, arma::zeros(1,1));
      map_.at(j) = q_.at(0) + r * std::cos(phi + q_.at(2));
      map_.at(j+1) = q_.at(1) + r * std::sin(phi + q_.at(2));
    }

    arma::mat calculate_z(double mx, double my) {
      return {{std::sqrt(std::pow(mx - q_.at(0), 2) + std::pow(my - q_.at(1), 2)),std::atan2(my - q_.at(1), mx - q_.at(0)) - q_.at(2)}};
    }

    /// @brief When a new obstacle estimate is received, update the current, previous,
    /// and delta configuration variables. Also update the map, and update the H matrix.
    void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg) 
    {
      // initialize some variables we'll be using
      double x, y;
      arma::mat Hj;

      // if the map is empty, initialize it and the xi vector because it means
      // it's the first time we're getting measurements
      if (map_.is_empty()) {
        initialize_map(msg);
        initialize_xi();
      }

      // update the state variables
      q_prev_ = q_;
      q_delta_ = q_ - q_prev_;
      xi_ = arma::join_vert(q_, map_);

      for (int j = 0; j < static_cast<int>(msg.markers.size()); j++) {
        // make some useful variables
        x = msg.markers.at(j).pose.position.x;
        y = msg.markers.at(j).pose.position.y;

        if (std::find(map_ids_.begin(), map_ids_.end(), msg.markers.at(j).id) == map_ids_.end()) {
          add_landmark_to_map(x, y, j, msg);
        }

        // update the measurement model
        Hj = measurement_model(x,y,j);

        // perform the SLAM update
        arma::mat xi_hat;
        if (count_ == 0) {
          arma::mat sigma_minus = A_ * sigma_ * A_.t() + Qbar_;
          arma::mat K = sigma_minus * Hj.t() * arma::inv(Hj * sigma_minus * Hj.t() + R_);
          xi_hat = xi_ + K * 
                (arma::mat({std::sqrt(std::pow(xi_.at(2 + j*2) - q_.at(0), 2) + std::pow(xi_.at(2 + j*2 + 1) - q_.at(1), 2)),
                turtlelib::normalize_angle(std::atan2(xi_.at(2+j*2+1) - q_.at(1), xi_.at(2 + j*2) - q_.at(0)) - q_.at(2))}) -
                arma::mat({std::sqrt(std::pow(x - q_.at(0), 2) + std::pow(y - q_.at(1), 2)), 
                turtlelib::normalize_angle(std::atan2(y - q_.at(1), x - q_.at(0)) - q_.at(2))}));
          sigma_ = (arma::eye(2,2) - K * Hj) * sigma_minus;
        } else {
          arma::mat sigma_minus = A_ * sigma_ * A_.t() + Qbar_;
          arma::mat K = sigma_minus * Hj.t() * arma::inv(Hj * sigma_minus * Hj.t() + R_);
          xi_hat = xi_ + K * 
                (arma::mat({std::sqrt(std::pow(xi_.at(2 + j*2) - q_.at(0), 2) + std::pow(xi_.at(2 + j*2 + 1) - q_.at(1), 2)),
                turtlelib::normalize_angle(std::atan2(xi_.at(2+j*2+1) - q_.at(1), xi_.at(2 + j*2) - q_.at(0)) - q_.at(2))}) -
                arma::mat({std::sqrt(std::pow(x - q_.at(0), 2) + std::pow(y - q_.at(1), 2)), 
                turtlelib::normalize_angle(std::atan2(y - q_.at(1), x - q_.at(0)) - q_.at(2))}));
          sigma_ = (arma::eye(2,2) - K * Hj) * sigma_minus;
        }

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = get_clock()->now();
      t.header.frame_id = "map";
      t.child_frame_id = "green/odom";

      t.transform.translation.x = xi_hat.at(0);
      t.transform.translation.y = xi_hat.at(1);
      t.transform.translation.z = 0.0;

      tf2::Quaternion q_tf2;
      q_tf2.setRPY(0, 0, q_.at(2));

      geometry_msgs::msg::Quaternion q = tf2::toMsg(q_tf2);

      t.transform.rotation.x = q.x;
      t.transform.rotation.y = q.y;
      t.transform.rotation.z = q.z;
      t.transform.rotation.w = q.w;

      tf_broadcaster_->sendTransform(t);
      }

    }

    arma::mat measurement_model(double mx, double my, int j)
    {
      // there already is noise from the sensor data here, so I don't need to
      // add more do I?
      double delta_x = mx - q_.at(0);
      double delta_y = my - q_.at(1);
      double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);

      arma::mat block1 = {{0, -delta_x/std::sqrt(d), -delta_y/std::sqrt(d)},
                          {-1, delta_y/std::sqrt(d), -delta_x/std::sqrt(d)}};
      arma::mat block2 = arma::join_vert(arma::zeros(1,2*(j-1)),arma::zeros(1,2*(j-1)));
      arma::mat block3 = {{delta_x/std::sqrt(d), delta_y/std::sqrt(d)},
                          {-delta_y/std::sqrt(d), delta_x/std::sqrt(d)}};
      arma::mat block4 = arma::join_vert(arma::zeros(1,2*n_-2*j),arma::zeros(1,2*n_-2*j));

      arma::mat H = arma::join_horiz(block1, block2, block3, block4);

      return H;           
    }
    
    void odometry_callback(const nav_msgs::msg::Odometry & msg)
    {
      // BEGIN CITATION [29]
      tf2::Quaternion tf2_quat;
      tf2::Matrix3x3 m(tf2_quat);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      // END CITATION [29]
      q_.at(0) = msg.pose.pose.position.x;
      q_.at(1) = msg.pose.pose.position.y;
      q_.at(2) = yaw;
    }

    arma::mat initialize_Q()
    {
      arma::mat Q = arma::eye(3,3);
      Q *= Qfactor_;
      Q = arma::join_vert(Q, arma::zeros(2*n_, 3));
      Q = arma::join_horiz(Q, arma::zeros(3 + 2*n_, 2*n_));
      return Q;
    }

    arma::mat initialize_sigma()
    {
      arma::mat sigma_q = arma::zeros(3,3);
      arma::mat sigma_m = arma::mat(2*n_, 2*n_, arma::fill::value(1000000));
      arma::mat sigma_qz = arma::join_vert(sigma_q, arma::zeros(2*n_,3));
      arma::mat sigma_mz = arma::join_vert(arma::zeros(3, 2*n_), sigma_m);

      return arma::join_horiz(sigma_qz, sigma_mz);
    }

    arma::mat create_A_matrix()
    {
      arma::mat A;
      if (std::abs(q_.at(2) - q_prev_.at(2)) < 1e-3) {
        arma::mat tmp = {{0,0,0},
                         {-q_delta_.at(0) * std::sin(q_prev_.at(2)),0,0},
                         {q_delta_.at(0) * std::cos(q_prev_.at(2)),0,0}};
        tmp = arma::join_vert(tmp, arma::zeros(2*n_, 3));
        A = arma::join_horiz(tmp,arma::zeros(3+2*n_,2*n_));
        A = arma::eye(n_,n_) + A;
      } else {
        arma::mat tmp = {{0,0,0},
                         {-q_delta_.at(0) / q_delta_.at(2) * std::cos(q_prev_.at(2)) + q_delta_.at(0)/q_delta_.at(2) * std::cos(q_prev_.at(2) + q_delta_.at(2)),0,0},
                         {-q_delta_.at(0) / q_delta_.at(2) * std::sin(q_prev_.at(2)) + q_delta_.at(0)/q_delta_.at(2) * std::sin(q_prev_.at(2) + q_delta_.at(2)),0,0}};
        tmp = arma::join_vert(tmp, arma::zeros(2*n_, 3));
        A = arma::join_horiz(tmp,arma::zeros(3+2*n_,2*n_));
        A = arma::eye(n_,n_) + A;
      }

      return A;
    }

    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
    
    // transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    arma::mat q_;
    arma::mat q_prev_;
    arma::mat q_delta_;
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
