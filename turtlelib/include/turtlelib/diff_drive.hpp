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

        /// \brief subtract this phi with another and store the result 
        /// in this object
        /// \param phi_new - the first vector to apply
        /// \return a reference to the newly transformed operator
        WheelPhi & operator-=(const WheelPhi & phi_new);

    };

    struct Config
    {
        double x = 0.0;
        double y = 0.0;
        double phi = 0.0;
    };

    /// \brief Subtract two phis from each other
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two vectors
    Vector2D operator-(WheelPhi lhs, const WheelPhi & rhs);

    class DiffDrive
    {
        private:
            double d=0.0;
            double r=0.0;

            WheelPhi phi;

            Config q;


        public:

            DiffDrive();

            explicit DiffDrive(double dist, double radius);

            explicit DiffDrive(double dist, double radius, WheelPhi phihat, Config qhat);

            Config ForwardKin(WheelPhi newphi);

            Vector2D InvKIn(Twist V); 


    };
}



# endif

