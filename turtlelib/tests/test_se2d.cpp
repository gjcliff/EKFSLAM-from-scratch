#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vector>
#include <sstream>
#include <iostream>

namespace turtlelib{
        TEST_CASE("Test the default constructor", "[default_constructor]"){
            Transform2D t;
            REQUIRE_THAT(t.rotation(), Catch::Matchers::WithinAbs(0, 0.001));
            REQUIRE_THAT(t.translation().x, Catch::Matchers::WithinAbs(0, 0.001));
            REQUIRE_THAT(t.translation().y, Catch::Matchers::WithinAbs(0, 0.001));
        }

        TEST_CASE("Test the constructor with only a Vector2D argument", "[Vector2D_constructor]"){
            Vector2D v = {1, 1};
            Transform2D t(v);
            REQUIRE_THAT(t.rotation(), Catch::Matchers::WithinAbs(0, 0.001));
            REQUIRE_THAT(t.translation().x, Catch::Matchers::WithinAbs(1, 0.001));
            REQUIRE_THAT(t.translation().y, Catch::Matchers::WithinAbs(1, 0.001));
        }

        TEST_CASE("Test the constructor with only a double argument", "[double_constructor]"){
            double radians = deg2rad(180);
            Transform2D t(radians);
            REQUIRE_THAT(t.rotation(), Catch::Matchers::WithinAbs(3.1415, 0.0001));
            REQUIRE_THAT(t.translation().x, Catch::Matchers::WithinAbs(0, 0.001));
            REQUIRE_THAT(t.translation().y, Catch::Matchers::WithinAbs(0, 0.001));
        }

        TEST_CASE("Test the constructor with both double and Vector2D arguments.", "[double_Vector2D_constructor]"){
            Vector2D v = {1, 1};
            double radians = deg2rad(180);
            Transform2D t(v, radians);
            REQUIRE_THAT(t.rotation(), Catch::Matchers::WithinAbs(3.1415, 0.0001));
            REQUIRE_THAT(t.translation().x, Catch::Matchers::WithinAbs(1, 0.001));
            REQUIRE_THAT(t.translation().y, Catch::Matchers::WithinAbs(1, 0.001));
        }

        TEST_CASE("Test the () operator on Point2D objects", "[operator() Point2D]"){
            Point2D p = {0,1};

            double degrees = 90;
            double radians = deg2rad(degrees);
            Transform2D transform(radians);

            p = transform(p);

            // SUCCEED();

            REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(-1, 0.001));
            REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(0, 0.001));
        }

        TEST_CASE("Test the () operator on Vector2D objects", "[operator() Vector2D]"){
            Vector2D v = {0,1};

            double degrees = 90;
            double radians = deg2rad(degrees);
            Transform2D transform(radians);

            v = transform(v);

            REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(-1, 0.001));
            REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(0, 0.001));
        }

        TEST_CASE("Test the () operator on Twist2D object", "[operator() Twist2D]"){
            Twist2D t = {1,1,1};

            double degrees = -90;
            double radians = deg2rad(degrees);
            Vector2D trans = {0, 1};

            Transform2D transform(trans, radians);
            // notice that this is the first test case demo'd in task B.6

            t = transform(t);

            REQUIRE_THAT(t.omega, Catch::Matchers::WithinAbs(1, 0.001));
            REQUIRE_THAT(t.x, Catch::Matchers::WithinAbs(0, 0.001));
            REQUIRE_THAT(t.y, Catch::Matchers::WithinAbs(1, 0.001));
        }

        TEST_CASE("TEST the inv() function", "[inv]"){
            double degrees = -90.0;
            double radians = deg2rad(degrees);
            Vector2D trans = {0.0,1.0};
            // std::vector<std::vector<double>> t = {{std::cos(radians),-std::sin(radians),trans.x},
            //                                       {std::sin(radians),std::cos(radians),trans.y},
            //                                       {0,0,1}};
            Transform2D t(trans, radians);
            Transform2D t_inv = t.inv();

            Transform2D identity = t * t_inv;

            REQUIRE_THAT(identity.translation().x, Catch::Matchers::WithinAbs(0.0, 0.001));
            REQUIRE_THAT(identity.translation().y, Catch::Matchers::WithinAbs(0.0, 0.001));
            REQUIRE_THAT(identity.rotation(), Catch::Matchers::WithinAbs(0.0, 0.001));
        }

        TEST_CASE("Test the *= operator", "[operator*=]"){
            // the test for inv() is also sort of a test for this operator.

            Transform2D t; // create an identity transformation
            
            double degrees = 90;
            double radians = deg2rad(degrees);
            Transform2D t_90(radians);

            t_90 *= t;

            REQUIRE_THAT(t_90.rotation(), Catch::Matchers::WithinAbs(1.5708, 0.001));

        }

        TEST_CASE("Test the translation() function", "[translation]"){
            Vector2D trans = {0,1};
            Transform2D t(trans);

            REQUIRE_THAT(t.translation().x, Catch::Matchers::WithinAbs(0.0, 0.001));
            REQUIRE_THAT(t.translation().y, Catch::Matchers::WithinAbs(1.0, 0.001));
        }

        TEST_CASE("Test the rotation() function", "[rotation]"){
            double degrees = 180;
            double radians = deg2rad(degrees);
            Vector2D v = {1,1};

            Transform2D t(v, radians);

            REQUIRE_THAT(t.rotation(), Catch::Matchers::WithinAbs(3.1415, 0.001));
        }

        TEST_CASE("Test the << operator","[operator<<]"){
            Transform2D t;
            std::ostringstream oss;
            oss << t;
            REQUIRE(oss.str() == "deg: 0 x: 0 y: 0");
        }

        TEST_CASE("Test the >> operator", "[operator>>]"){
            double radians;
            Vector2D trans;

            SECTION("Test with spaces"){
                std::istringstream iss("1.5708 1 1");
                iss >> radians >> trans.x >> trans.y;
                REQUIRE(radians == 1.5708);
                REQUIRE(trans.x == 1);
                REQUIRE(trans.y == 1);
            }

            SECTION("Test with newline characters"){
                std::istringstream iss("1.5708\n1\n1\n");
                iss >> radians >> trans.x >> trans.y;
                REQUIRE(radians == 1.5708);
                REQUIRE(trans.x == 1);
                REQUIRE(trans.y == 1);
            }
        }

        TEST_CASE("Test * operator", "[operator*]"){
            Transform2D t; // create an identity transformation
            
            double degrees = 90;
            double radians = deg2rad(degrees);
            Transform2D t_90(radians);

            Transform2D t_out = t * t_90;
            REQUIRE_THAT(t_out.rotation(), Catch::Matchers::WithinAbs(1.5708, 0.001));
        }
}