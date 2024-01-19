#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vector>

namespace turtlelib{
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
        double degrees = -90;
        double radians = deg2rad(degrees);
        Vector2D trans = {0,1};
        // std::vector<std::vector<double>> t = {{std::cos(radians),-std::sin(radians),trans.x},
        //                                       {std::sin(radians),std::cos(radians),trans.y},
        //                                       {0,0,1}};
        Transform2D t(trans, radians);
        Transform2D t_inv = t.inv();

    }
}