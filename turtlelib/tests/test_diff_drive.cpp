#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iosfwd>
#include <iostream>
#include <sstream>

namespace turtlelib
{
TEST_CASE("Test FK no translation only", "[FK_translation]")
{
  DiffDrive turtlebot;
  turtlebot.FK(0.707, 0.707);
  Configuration q = turtlebot.get_current_configuration();

  REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(q.x, Catch::Matchers::WithinAbs(0.1414, 1e-5));
  REQUIRE_THAT(q.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Test FK no rotation only", "[FK_rotation]")
{
  DiffDrive turtlebot;
  turtlebot.FK(1.5708, 0.0);
  Configuration q = turtlebot.get_current_configuration();

  // REQUIRE_THAT(q.theta, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(q.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(q.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}
}
