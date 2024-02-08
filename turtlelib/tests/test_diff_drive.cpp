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
  double radians = deg2rad(90);
  Twist2D Vb = turtlebot.FK(radians, radians);
  Configuration q = turtlebot.update_configuration(Vb).at(0);

  RobotDimensions rd = turtlebot.get_robot_dimensions();
  double distance_traveled = 2 * PI * rd.r * (radians / deg2rad(360));

  REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(q.x, Catch::Matchers::WithinAbs(distance_traveled, 1e-5));
  REQUIRE_THAT(q.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Test FK translation backward only", "[FK_translation_backward]")
{
  DiffDrive turtlebot;
  double radians = deg2rad(-90);
  Twist2D Vb = turtlebot.FK(radians, radians);
  Configuration q = turtlebot.update_configuration(Vb).at(0);

  RobotDimensions rd = turtlebot.get_robot_dimensions();
  double distance_traveled = 2 * PI * rd.r * (radians / deg2rad(360));

  REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(q.x, Catch::Matchers::WithinAbs(distance_traveled, 1e-5));
  REQUIRE_THAT(q.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Test FK rotation CW only", "[FK_rotation_CW]")
{
  DiffDrive turtlebot;
  double radians_l = deg2rad(90);
  double radians_r = 0.0;
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q = turtlebot.update_configuration(Vb).at(0);

  RobotDimensions rd = turtlebot.get_robot_dimensions();

  // assuming when the left wheel turns and the right wheel is still, this
  // will rotate the robot around the point where the right wheel is on the
  // ground. I can calculate the arc length of this turn by how much the left
  // wheel traveled in the x direction, and turn that into the new theta of the
  // robot's configuration in the world frame.
  double arc_length = 2 * PI * rd.r * (radians_l / deg2rad(360));
  // arc_length = theta/360
  // negative sign is crucial, because CW rotation is negative
  // the radius of this larger circle is actually 2 * D, since center of rotation
  // is at where the right wheel touches the ground.
  double expected_q_theta = -arc_length * deg2rad(360) / (2 * PI * (2 * rd.D));

  REQUIRE(q.x > 0.0);
  REQUIRE(q.y < 0.0);
  REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(expected_q_theta, 1e-5));
}

TEST_CASE("Test FK rotation CCW only", "[FK_rotation_CCW]")
{
  DiffDrive turtlebot;
  double radians_l = 0.0;
  double radians_r = deg2rad(90);
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q = turtlebot.update_configuration(Vb).at(0);

  RobotDimensions rd = turtlebot.get_robot_dimensions();

  // assuming when the left wheel turns and the right wheel is still, this
  // will rotate the robot around the point where the right wheel is on the
  // ground. I can calculate the arc length of this turn by how much the left
  // wheel traveled in the x direction, and turn that into the new theta of the
  // robot's configuration in the world frame.
  double arc_length = 2 * PI * rd.r * (radians_r / deg2rad(360));
  // arc_length = theta/360
  // expected q is positive here because we are rotating CCW
  // the radius of this larger circle is actually 2 * D, since center of rotation
  // is at where the right wheel touches the ground.
  double expected_q_theta = arc_length * deg2rad(360) / (2 * PI * (2 * rd.D));

  REQUIRE(q.x > 0.0);
  REQUIRE(q.y > 0.0);
  REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(expected_q_theta, 1e-5));
}

TEST_CASE("Test FK rotation and translation", "[FK_rotation_translation]")
{
  DiffDrive turtlebot;
  double radians_l = deg2rad(90);
  double radians_r = deg2rad(180);
  Twist2D Vb = turtlebot.FK(radians_l, radians_r);
  Configuration q = turtlebot.update_configuration(Vb).at(0);

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
  Configuration q = turtlebot.update_configuration(Vb).at(0);

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
