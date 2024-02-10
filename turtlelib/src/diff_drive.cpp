#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <vector>

namespace turtlelib
{

DiffDrive::DiffDrive()
{
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(double phi_right, double phi_left)
{
  phi_r = phi_right;
  phi_l = phi_left;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(RobotDimensions rd)
{
  rd.L = rd.L;
  rd.D = rd.D;
  rd.r = rd.r;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(Configuration q_orig)
{
  q = q_orig;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(Configuration q_orig, RobotDimensions rd)
{
  q = q_orig;
  rd.L = rd.L;
  rd.D = rd.D;
  rd.r = rd.r;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

Twist2D DiffDrive::FK(double phi_l_p, double phi_r_p)
{
  // double d_phi_r = phi_r_p - phi_r;
  // double d_phi_l = phi_l_p - phi_l;
  vector<double> u{phi_l_p, phi_r_p};
  vector<double> Vb_mat(vector<double>(3, 0.0));

  // calculate Vb based on the pseudo inverse of the H matrix
  // and the input controls
  for (int i = 0; i < (int)H_pseudo.size(); i++) {
    for (int j = 0; j < (int)H_pseudo.at(0).size(); j++) {
      Vb_mat[i] += H_pseudo[i][j] * u[j];
    }
  }

  phi_r += phi_r_p;
  phi_l += phi_l_p;

  return {Vb_mat.at(0), Vb_mat.at(1), Vb_mat.at(2)};
}

vector<Configuration> DiffDrive::update_configuration(Twist2D Vb)
{
  Transform2D Tbb_prime = integrate_twist(Vb);

  // now, compute the transformation matrix of the robot body frame {b} in
  // the world frame {w}
  Transform2D Twb({q.x, q.y}, q.theta);
  Transform2D Twb_prime = Twb * Tbb_prime;

  Configuration q_dot{Twb_prime.rotation(), Twb_prime.translation().x, Twb_prime.translation().y};

  q.theta = q_dot.theta;
  q.x = q_dot.x;
  q.y = q_dot.y;

  return {q, q_dot};
}

vector<double> DiffDrive::IK(Twist2D twist)
{
  vector<double> phi_delta(2, 0.0);
  vector<double> Vb = {twist.omega, twist.x, twist.y};

  double threshold = 1e-10;
  if (twist.y >= threshold || twist.y <= -threshold) {
    throw std::logic_error("wheels are slipping!! Vb.y is not 0");
  }

  // multiple the H matrix by the body twist to find the new wheel velocities
  for (int i = 0; i < (int)H.size(); i++) {
    for (int j = 0; j < (int)H.at(0).size(); j++) {
      phi_delta[i] += H[i][j] * Vb[j];
    }
  }

  return phi_delta;
}

Configuration DiffDrive::get_current_configuration() const
{
  return q;
}

void DiffDrive::set_current_configuration(Configuration q_new)
{
  q = q_new;
}

RobotDimensions DiffDrive::get_robot_dimensions() const
{
  return rd;
}

void DiffDrive::set_robot_dimensions(RobotDimensions rd_new)
{
  rd = rd_new;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

vector<vector<double>> DiffDrive::construct_H_matrix()
{
  vector<vector<double>> H_tmp = {{-rd.D, 1, 0},
    {rd.D, 1, 0}};

  for (int i = 0; i < (int)H_tmp.size(); i++) {
    for (int j = 0; j < (int)H_tmp.at(0).size(); j++) {
      H_tmp[i][j] *= 1 / rd.r; // r/3???
    }
  }

  return H_tmp;
}

vector<vector<double>> DiffDrive::construct_H_pseudo_matrix()
{
  // I calculated this using numpy, and am simply constructing the matrix here
  // in lieu of calculating the pseudo matrix by hand in C++ with no help from
  // libraries.
  vector<vector<double>> H_pseudo_tmp = {{-1 / rd.D, 1 / rd.D}, {1, 1}, {0, 0}};

  for (int i = 0; i < (int)H_pseudo_tmp.size(); i++) {
    for (int j = 0; j < (int)H_pseudo_tmp.at(0).size(); j++) {
      H_pseudo_tmp[i][j] *= rd.r / 2;
    }
  }

  return H_pseudo_tmp;
}
}
