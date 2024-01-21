#include"turtlelib/svg.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/geometry2d.hpp"
#include<catch2/catch_test_macros.hpp>
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include<string>

namespace turtlelib{
    TEST_CASE("Test that svg class can draw a point", "[point]")
    {
        Svg svg("point_test.svg");
        Point2D p = {0,1};
        svg.draw_point(p);
        svg.close();

        bool same = true;

        std::ifstream test_file("point_test.svg");
        std::ifstream test_file_compare("point_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a vector without specifying a starting point", "[vector_no_start]")
    {
        Svg svg("vector_no_start_test.svg");
        Vector2D v = {0, 1};
        svg.draw_vector(v);
        svg.close();

        bool same = true;

        std::ifstream test_file("vector_no_start_test.svg");
        std::ifstream test_file_compare("vector_no_start_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a vector when specifying a starting point", "[vector_start]")
    {
        Svg svg("vector_start_test.svg");
        Vector2D v = {0, 1};
        Point2D tail = {2,2};
        svg.draw_vector(v, tail);
        svg.close();

        bool same = true;

        std::ifstream test_file("vector_start_test.svg");
        std::ifstream test_file_compare("vector_start_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a coordinate frame without specifying a start point", "[coordinate_frame_default]")
    {
        Svg svg("coordinate_frame_test_default.svg");

        svg.draw_coordiante_frame();
        svg.close();

        bool same = true;

        std::ifstream test_file("coordinate_frame_test_default.svg");
        std::ifstream test_file_compare("coordinate_frame_test_default_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a coordinate frame while specifying a start point", "[coordinate_frame_point]")
    {
        Svg svg("coordinate_frame_test_point.svg");

        Point2D p = {1,1};
        svg.draw_coordiante_frame(p);
        svg.close();

        bool same = true;

        std::ifstream test_file("coordinate_frame_test_point.svg");
        std::ifstream test_file_compare("coordinate_frame_test_point_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a coordinate frame while specifying a start point and specifying name", "[coordinate_frame_point_text]")
    {
        Svg svg("coordinate_frame_test_point_text.svg");

        Point2D p = {1,1};
        svg.draw_coordiante_frame(p, "b");
        svg.close();

        bool same = true;

        std::ifstream test_file("coordinate_frame_test_point_text.svg");
        std::ifstream test_file_compare("coordinate_frame_test_point_text_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }
}