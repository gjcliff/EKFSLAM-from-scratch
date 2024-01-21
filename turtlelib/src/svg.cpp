#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/svg.hpp"
#include<fstream>
#include<string>

namespace turtlelib
{
    Svg::Svg(){
        svgFile.open("default.svg", std::ios_base::out | std::ios_base::trunc);
        svgFile << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;
        svgFile << "<defs>" << std::endl;
        svgFile << "  <marker" << std::endl;
        svgFile << "    style=\"overflow:visible\"" << std::endl;
        svgFile << "    id=\"Arrow1Sstart\"" << std::endl;
        svgFile << "    refX=\"0.0\"" << std::endl;
        svgFile << "    refY=\"0.0\"" << std::endl;
        svgFile << "    orient=\"auto\">" << std::endl;
        svgFile << "      <path" << std::endl;
        svgFile << "        transform=\"scale(0.2) translate(6,0)\"" << std::endl;
        svgFile << "        style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"" << std::endl;
        svgFile << "        d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"" << std::endl;
        svgFile << "        />" << std::endl;
        svgFile << "    </marker>" << std::endl;
        svgFile << "</defs>";
    }

    Svg::Svg(std::string fileName)
    {
        svgFile.open(fileName, std::ios_base::out | std::ios_base::trunc);
        svgFile << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;
    }

    void Svg::draw_point(Point2D p)
    {
        svgFile << "<circle cx=\"" << p.x << "\" cy=\"" << p.y << "\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\"/>" << std::endl;
    }

    void Svg::draw_vector(Vector2D v, Point2D tail)
    {
        Point2D head = tail + v;
        svgFile << "<line x1=\"" << tail.x << "\" x2=\"" << head.x << "\" y1=\"" << tail.y << "\" y2=\"" << head.y << "\" stroke=\"purple\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
    }

    void Svg::draw_coordiante_frame(Vector2D x, Vector2D y, Point2D tail, std::string text)
    {
        Point2D headx = tail + x;
        Point2D heady = tail + y;
        svgFile << "<g>" << std::endl;
        svgFile << "<line x1=\"" << headx.x << "\" x2=\"" << tail.x << "\" y1=\"" << headx.y << "\" y2=\"" << tail.y << "stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<line x1=\"" << heady.x << "\" x2=\"" << tail.x << "\" y1=\"" << heady.y << "\" y2=\"" << tail.y << "stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<text x=\"" << tail.x << "\" y=\"" << tail.y + 0.25 << "\">{" << text << "}</text>" << std::endl;
        svgFile << "</g>" << std::endl;
    }   

    void Svg::close(){
        svgFile << "</svg>";
        // no need to explicityly call close, ofstream destructors automatically
        // close the underlying file. - Professional C++
    }
}