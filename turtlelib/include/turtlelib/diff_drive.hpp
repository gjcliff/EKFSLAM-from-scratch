#ifndef TURTLELIB_DIFFDRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFFDRIVE_HPP_INCLUDE_GUARD
/// @file
/// @brief Model the kinematics of a differential drive robot.


#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <vector>

namespace turtlelib
{

/// @brief a robot configuration in the world frame
struct Configuration
{
  /// @brief the x coordinate
  double x = 0.0;

  /// @brief the y coordinate
  double y = 0.0;

  /// @brief the angle
  double theta = 0.0;
};

class DiffDrive
{
public:
  /// @brief create a model of the kinematics of a differential drive robot
  DiffDrive();

  /// @brief create a model of the kinematics of a differential drive robot
  /// that specifies the initial positions of the robot's wheels
  /// @param phi_right - the position of the robot's right wheel
  /// @param phi_left - the position of the robot's left wheel
  DiffDrive(double phi_right, double phi_left);

  /// @brief create the model of the kinematics of a differential drive robot
  /// that specifies the positions of the robot's wheels in relation to its
  /// body frame at the center of the robot
  /// @param length - the length of the robot, divided by two
  /// @param width - the width of the robot, divided by two
  /// @param r - the radius of the wheels of the robot
  DiffDrive(double length, double width, double radius);

  /// @brief create a model of the kinematics of a differential drive robot
  /// that specifies the initial configuration of the robot
  /// @param q_orig - the original configuration of the robot
  DiffDrive(Configuration q_orig);

  /// @brief create a model of the kinematics of a differential drive robot
  /// that specifies the initial configuration of the robot and additionally
  /// the positions of the robot's wheels in relation to its body frame at the
  /// center of the robot
  /// @param q_orig - the original configuration of the robot
  /// @param L - the length of the robot, divided by two
  /// @param D - the width of the robot, divided by two
  /// @param r - the radius of the wheels of the robot
  DiffDrive(Configuration q_orig, double length, double width, double radius);

  /// @brief given new wheel positions, update the configuration
  /// @param phi_r_p
  /// @param phi_l_p
  void FK(double phi_r_p, double phi_l_p);

  /// @brief compute the wheel velocities required to make the robot move
  /// at a given body twist
  /// @param twist
  void IK(Twist2D twist);

  /// @brief retrieve the robot's current configuration
  /// @return the robot's current configuration
  Configuration get_current_configuration();

  /// @brief construct the H matrix
  /// @return a 2x3 H matrix
  vector<vector<double>> construct_H_matrix();

  /// @brief construct the pseudo H matrix
  /// @return a 3x2 pseudo H matrix
  vector<vector<double>> construct_H_pseudo_matrix();

private:
  double phi_r = 0.0;
  double phi_l = 0.0;
  double L = 1.0;
  double D = 1.0;
  double r = 0.1;
  Transform2D Tb1 = Transform2D({L, D}, 0.0);
  Transform2D Tb2 = Transform2D({L, -D}, 0.0);
  Configuration q;
  vector<vector<double>> H;
  vector<vector<double>> H_pseudo;
};
}

#endif
