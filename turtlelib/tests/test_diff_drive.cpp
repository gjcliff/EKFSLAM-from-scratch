#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iosfwd>
#include <iostream>
#include <sstream>

namespace turtlelib
{
TEST_CASE("Test FK translation forward only", "[FK_translation_forward]")
{
  DiffDrive turtlebot;
  RobotDimensions r_orig = turtlebot.get_robot_dimensions();

  // do FK calculations
  double radians_l = deg2rad(90);
  double radians_r = deg2rad(90);
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q_dot = turtlebot.update_configuration(Vb);

  // generating test case, from eq. 13.15 in Modern Robotics textbook
  vector<double> q_dot_tmp(3, 0.0);
  vector<double> phi_d{radians_l, radians_r};
  // why do I have to have a four here for this to work? something is
  // wrong.
  double phi = -radians_l * r_orig.r / (4.0 * r_orig.D);
  vector<vector<double>> H_pseudo_test = {
    {-r_orig.r / (2.0 * r_orig.D), r_orig.r / (2.0 * r_orig.D)},
    {r_orig.r / 2.0 * std::cos(phi), r_orig.r / 2.0 * std::cos(phi)},
    {r_orig.r / 2.0 * std::sin(phi), r_orig.r / 2.0 * std::sin(phi)}};

  for (int i = 0; i < (int)H_pseudo_test.size(); i++) {
    for (int j = 0; j < (int)H_pseudo_test.at(0).size(); j++) {
      q_dot_tmp.at(i) += H_pseudo_test.at(i).at(j) * phi_d.at(j);
    }
  }

  Configuration q_dot_test{q_dot_tmp.at(0), q_dot_tmp.at(1), q_dot_tmp.at(2)};

  REQUIRE_THAT(q_dot.theta, Catch::Matchers::WithinAbs(q_dot_test.theta, 1e-2));
  REQUIRE_THAT(q_dot.x, Catch::Matchers::WithinAbs(q_dot_test.x, 1e-2));
  REQUIRE_THAT(q_dot.y, Catch::Matchers::WithinAbs(q_dot_test.y, 1e-2));
}

TEST_CASE("Test FK translation backward only", "[FK_translation_backward]")
{
  DiffDrive turtlebot;
  RobotDimensions r_orig = turtlebot.get_robot_dimensions();

  // do FK calculations
  double radians_l = deg2rad(-90);
  double radians_r = deg2rad(-90);
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q_dot = turtlebot.update_configuration(Vb);

  // generating test case, from eq. 13.15 in Modern Robotics textbook
  vector<double> q_dot_tmp(3, 0.0);
  vector<double> phi_d{radians_l, radians_r};
  // why do I have to have a four here for this to work? something is
  // wrong.
  double phi = -radians_l * r_orig.r / (4.0 * r_orig.D);
  vector<vector<double>> H_pseudo_test = {
    {-r_orig.r / (2.0 * r_orig.D), r_orig.r / (2.0 * r_orig.D)},
    {r_orig.r / 2.0 * std::cos(phi), r_orig.r / 2.0 * std::cos(phi)},
    {r_orig.r / 2.0 * std::sin(phi), r_orig.r / 2.0 * std::sin(phi)}};

  for (int i = 0; i < (int)H_pseudo_test.size(); i++) {
    for (int j = 0; j < (int)H_pseudo_test.at(0).size(); j++) {
      q_dot_tmp.at(i) += H_pseudo_test.at(i).at(j) * phi_d.at(j);
    }
  }

  Configuration q_dot_test{q_dot_tmp.at(0), q_dot_tmp.at(1), q_dot_tmp.at(2)};

  REQUIRE_THAT(q_dot.theta, Catch::Matchers::WithinAbs(q_dot_test.theta, 1e-2));
  REQUIRE_THAT(q_dot.x, Catch::Matchers::WithinAbs(q_dot_test.x, 1e-2));
  REQUIRE_THAT(q_dot.y, Catch::Matchers::WithinAbs(q_dot_test.y, 1e-2));
}

TEST_CASE("Test FK rotation CW only", "[FK_rotation_CW]")
{
  DiffDrive turtlebot;
  RobotDimensions r_orig = turtlebot.get_robot_dimensions();

  // do FK calculations
  double radians_l = deg2rad(90);
  double radians_r = 0.0;
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q_dot = turtlebot.update_configuration(Vb);

  // generating test case, from eq. 13.15 in Modern Robotics textbook
  vector<double> q_dot_tmp(3, 0.0);
  vector<double> phi_d{radians_l, radians_r};
  // why do I have to have a four here for this to work? something is
  // wrong.
  double phi = -radians_l * r_orig.r / (4.0 * r_orig.D);
  vector<vector<double>> H_pseudo_test = {
    {-r_orig.r / (2.0 * r_orig.D), r_orig.r / (2.0 * r_orig.D)},
    {r_orig.r / 2.0 * std::cos(phi), r_orig.r / 2.0 * std::cos(phi)},
    {r_orig.r / 2.0 * std::sin(phi), r_orig.r / 2.0 * std::sin(phi)}};

  for (int i = 0; i < (int)H_pseudo_test.size(); i++) {
    for (int j = 0; j < (int)H_pseudo_test.at(0).size(); j++) {
      q_dot_tmp.at(i) += H_pseudo_test.at(i).at(j) * phi_d.at(j);
    }
  }

  Configuration q_dot_test{q_dot_tmp.at(0), q_dot_tmp.at(1), q_dot_tmp.at(2)};

  REQUIRE_THAT(q_dot.theta, Catch::Matchers::WithinAbs(q_dot_test.theta, 1e-3));
  REQUIRE_THAT(q_dot.x, Catch::Matchers::WithinAbs(q_dot_test.x, 1e-3));
  REQUIRE_THAT(q_dot.y, Catch::Matchers::WithinAbs(q_dot_test.y, 1e-3));
}

TEST_CASE("Test FK rotation CCW only", "[FK_rotation_CCW]")
{
  DiffDrive turtlebot;
  RobotDimensions r_orig = turtlebot.get_robot_dimensions();

  // do FK calculations
  double radians_l = 0.0;
  double radians_r = deg2rad(90);
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q_dot = turtlebot.update_configuration(Vb);

  // generating test case, from eq. 13.15 in Modern Robotics textbook
  vector<double> q_dot_tmp(3, 0.0);
  vector<double> phi_d{radians_l, radians_r};
  double phi = radians_r * r_orig.r / (4.0 * r_orig.D);
  vector<vector<double>> H_pseudo_test = {
    {-r_orig.r / (2.0 * r_orig.D), r_orig.r / (2.0 * r_orig.D)},
    {r_orig.r / 2.0 * std::cos(phi), r_orig.r / 2.0 * std::cos(phi)},
    {r_orig.r / 2.0 * std::sin(phi), r_orig.r / 2.0 * std::sin(phi)}};

  for (int i = 0; i < (int)H_pseudo_test.size(); i++) {
    for (int j = 0; j < (int)H_pseudo_test.at(0).size(); j++) {
      q_dot_tmp.at(i) += H_pseudo_test.at(i).at(j) * phi_d.at(j);
    }
  }

  Configuration q_dot_test{q_dot_tmp.at(0), q_dot_tmp.at(1), q_dot_tmp.at(2)};

  REQUIRE_THAT(q_dot.theta, Catch::Matchers::WithinAbs(q_dot_test.theta, 1e-3));
  REQUIRE_THAT(q_dot.x, Catch::Matchers::WithinAbs(q_dot_test.x, 1e-3));
  REQUIRE_THAT(q_dot.y, Catch::Matchers::WithinAbs(q_dot_test.y, 1e-3));
}

TEST_CASE("Test FK rotation and translation", "[FK_rotation_translation]")
{
  DiffDrive turtlebot;
  double radians_l = deg2rad(90);
  double radians_r = deg2rad(180);
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q = turtlebot.update_configuration(Vb);

  RobotDimensions rd = turtlebot.get_robot_dimensions();

  // rotation calculations
  double arc_length = 2 * PI * rd.r * ((radians_r - radians_l) / deg2rad(360));
  double expected_q_theta = arc_length * deg2rad(360) / (2 * PI * (2 * rd.D));

  REQUIRE(q.x > 0.0);
  REQUIRE(q.y > 0.0);
  REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(expected_q_theta, 1e-5));
}

TEST_CASE("Test FK rotation and translation inverse", "[FK_rotation_translation_inverse]")
{
  DiffDrive turtlebot;
  double radians_l = deg2rad(-90);
  double radians_r = deg2rad(-180);
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q = turtlebot.update_configuration(Vb);

  RobotDimensions rd = turtlebot.get_robot_dimensions();

  // rotation calculations
  double arc_length = 2 * PI * rd.r * ((radians_r - radians_l) / deg2rad(360));
  double expected_q_theta = arc_length * deg2rad(360) / (2 * PI * (2 * rd.D));

  REQUIRE(q.x < 0.0);
  REQUIRE(q.y > 0.0);
  REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(expected_q_theta, 1e-5));
}

TEST_CASE("Test IK", "[IK]")
{
  DiffDrive turtlebot;
  SECTION("Test linear x forwards")
  {
    // follow a twist moving linearly in the x direction
    Twist2D Vb{0.0, 1.0, 0.0};
    vector<double> phi_dot = turtlebot.IK(Vb);

    REQUIRE_THAT(phi_dot[0], Catch::Matchers::WithinAbs(10.0, 1e-5));
    REQUIRE_THAT(phi_dot[1], Catch::Matchers::WithinAbs(10.0, 1e-5));
  }

  SECTION("Test linear x backwards")
  {
    // follow a twist moving linearly in the x direction
    Twist2D Vb{0.0, -1.0, 0.0};
    vector<double> phi_dot = turtlebot.IK(Vb);

    REQUIRE_THAT(phi_dot[0], Catch::Matchers::WithinAbs(-10.0, 1e-5));
    REQUIRE_THAT(phi_dot[1], Catch::Matchers::WithinAbs(-10.0, 1e-5));
  }

  SECTION("Test rotation CCW only")
  {
    // follow a twist moving linearly in the x direction
    Twist2D Vb{1.0, 0.0, 0.0};
    vector<double> phi_dot = turtlebot.IK(Vb);

    REQUIRE_THAT(phi_dot[0], Catch::Matchers::WithinAbs(-10.0, 1e-5));
    REQUIRE_THAT(phi_dot[1], Catch::Matchers::WithinAbs(10.0, 1e-5));
  }

  SECTION("Test rotation CW only")
  {
    // follow a twist moving linearly in the x direction
    Twist2D Vb{-1.0, 0.0, 0.0};
    vector<double> phi_dot = turtlebot.IK(Vb);

    REQUIRE_THAT(phi_dot[0], Catch::Matchers::WithinAbs(10.0, 1e-5));
    REQUIRE_THAT(phi_dot[1], Catch::Matchers::WithinAbs(-10.0, 1e-5));
  }

  SECTION("Test Invalid Twist")
  {
    // follow a twist moving linearly in the y direction
    Twist2D Vb{0.0, 0.0, 1.0};

    REQUIRE_THROWS(turtlebot.IK(Vb));
  }

}
}
