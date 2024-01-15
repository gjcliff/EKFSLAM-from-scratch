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

    TEST_CASE("Test << operator for points", "[operator<<]"){
        Point2D point;
        point = {0, 1};

        std::ostringstream oss;
        oss << point;
        REQUIRE(oss.str() == "[0 1]");
    }

    TEST_CASE("Testing >> operator for points", "[operator>>]"){
        Point2D point;
        SECTION("test whether >> can handle and array input: '[0 1]'"){
            std::istringstream iss("[0 1]");
            iss >> point;   
            REQUIRE(point.x == 0);
            REQUIRE(point.y == 1);
        }

        SECTION("test wheter >> can handle an input of two doubles: '0 1'"){
            std::istringstream iss("0 1");
            iss >> point;   
            REQUIRE(point.x == 0);
            REQUIRE(point.y == 1);
        }
        
    }

    TEST_CASE("Testing - operator for points", "operator-"){
        Point2D head;
        Point2D tail;

        head = {10, 10};
        tail = {5, 5};

        Vector2D vec = head-tail;

        REQUIRE(vec.x == 5);
        REQUIRE(vec.y == 5);
    }

    TEST_CASE("Testing + operator for points", "operator+"){
        Point2D tail;
        Vector2D disp;
        
        tail = {0, 0};
        disp = {5, 5};

        Point2D head = tail + disp;

        REQUIRE(head.x == 5);
        REQUIRE(head.y == 5);
    }
    
    TEST_CASE("Testing << operator for vectors", "operator<< vector"){
        Vector2D vec;
        vec = {0, 1};

        std::ostringstream oss;
        oss << vec;
        REQUIRE(oss.str() == "[0 1]");
    }

    TEST_CASE("Testing >> operator for vectors", "operator>> vector"){
        Vector2D vec;
        SECTION("test whether >> can handle an input of two doubles: '[0 1]'"){
            std::istringstream iss("[0 1]");
            iss >> vec;
            REQUIRE(vec.x == 0);
            REQUIRE(vec.y == 1);
        }
        SECTION("test whether >> can handle an input of two doubles: '0 1'"){
            std::istringstream iss("0 1");
            iss >> vec;
            REQUIRE(vec.x == 0);
            REQUIRE(vec.y == 1);
        }
        
    }
}


