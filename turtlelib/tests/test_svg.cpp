#include"turtlelib/svg.hpp"
#include"turtlelib/geometry2d.hpp"
#include<catch2/catch_test_macros.hpp>
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include<string>

namespace turtlelib{
    TEST_CASE("Test that svg class can draw a line", "[line]"){
        Svg svg;
        Point2D p = {0,1};
        svg.draw_point(p);
        svg.close();
    }
}