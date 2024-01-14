#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>

using namespace turtlelib{
    TEST_CASE("normalize_angle works", "[normalize angle]") {
        REQUIRE(normalize_angle(PI) == PI);
        REQUIRE(normalize_angle(-PI) == -PI);
        REQUIRE(normalize_angle(0) == 0);
        REQUIRE(normalize_angle(-PI/4) ==  -PI/4);
        REQUIRE(normalize_angle(3*PI/2) == PI/2);
        REQUIRE(normalize_angle(-5*PI/2) == -PI/2);
    }
}


