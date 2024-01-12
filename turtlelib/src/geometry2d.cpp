#include "turtlelib/geometry2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <math.h>

namespace turtlelib
{
    double normalize_angle(double rad){
        while (rad > PI || rad < -PI){
            if (rad > PI){
                rad -= PI;
            } else if (rad < -PI){
                rad += PI;
            }
        }
        return rad;
    }

    

} // namespace turtlelib
