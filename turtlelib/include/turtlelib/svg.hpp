#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Draw points, vectors, and coordinate frames

#include<iosfwd> // contains forward definitions for iostream objects
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include<fstream>
#include<string>

namespace turtlelib
{

    /// \brief draw points and vectors in an svg file
    class Svg
    {
    public:
        /// \brief create an svg file with the default name, "default.svg",
        /// delete any previous contents, and open it for writing
        Svg();

        /// \brief create a svg file with a specific file name, delete its
        ///contents, and open it for writing
        /// \param fileName - the file name. MUST append .svg
        Svg(std::string fileName);

        /// \brief draw a point in the svg file
        /// \param p - the coordinates of the point that will be drawn
        void draw_point(Point2D p);

        /// \brief draw a vector in the svg file
        /// \param v - the vector that will be drawn
        void draw_vector(Vector2D v, Point2D tail);

        /// \brief draw a coordinate frame in the svg file
        /// \param t - the coordinate frame that will be drawn
        void draw_coordiante_frame(Vector2D x, Vector2D y, Point2D p, std::string text);

        void close();


    private:
        std::ofstream svgFile;
        Transform2D viewboxOrigin;
    };


}
#endif