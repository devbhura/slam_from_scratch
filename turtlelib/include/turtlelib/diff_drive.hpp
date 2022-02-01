#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include <math.h> 
#include "rigid2d.hpp"
/// \file
/// \brief 

namespace turtlelib
{

    struct WheelPhi
    {
        double left_phi=0.0;
        double right_phi=0.0;

    };

    class DiffDrive
    {
        private:
            double d=0.0;
            double r=0.0;

            WheelPhi phi;

            Transform2D T;


        public:

            DiffDrive();

            explicit DiffDrive(double dist, double radius);

            explicit DiffDrive(double dist, double radius, Transform2D transform);

            Twist getTwist(Vector2D u);

            Vector2D getwheelvel()

    };
}



# endif

