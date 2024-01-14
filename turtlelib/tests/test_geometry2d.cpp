#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <iosfwd>
#include <sstream>


namespace turtlelib{
    TEST_CASE("normalize_angle works", "[normalize angle]") {
        REQUIRE(normalize_angle(PI) == PI);
        REQUIRE(normalize_angle(-PI) == -PI);
        REQUIRE(normalize_angle(0) == 0);
        REQUIRE(normalize_angle(-PI/4) ==  -PI/4);
        REQUIRE(normalize_angle(3*PI/2) == PI/2);
        REQUIRE(normalize_angle(-5*PI/2) == -PI/2);
    }

    TEST_CASE("Test << operator", "[operator<<]"){
        Point2D point;
        point.x = 0;
        point.y = 1;

        std::ostringstream oss;
        oss << point;
        REQUIRE(oss.str() == "[" + std::to_string(point.x) + " " + std::to_string(point.y) + "]");
    }
}


