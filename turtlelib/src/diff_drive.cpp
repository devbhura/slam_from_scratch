#include "turtlelib/diff_drive.hpp"
#include <iostream>
#include <math.h> 
#include <stdexcept>

namespace turtlelib
{

// Struct WheelPhi
//  WheelPhi & WheelPhi::operator-=(const WheelPhi & phi_new)
//  {
//     left_phi -= phi_new.left_phi;
//     right_phi -= phi_new.right_phi;
    
//     return *this;
//  }

 // Struct related Operator - 
//  Vector2D operator-(WheelPhi lhs, const WheelPhi & rhs)
//  {
//     lhs-=rhs;
//     Vector2D v;
//     v.x = lhs.left_phi;
//     v.y = lhs.right_phi;

//     return v;
//  }

// Diff Drive
 DiffDrive::DiffDrive()
 {
    d = 0.0;
    r = 0.0;
    phi.right_phi = 0.0;
    phi.left_phi = 0.0;
    q.phi = 0.0;
    q.x = 0.0;
    q.y = 0.0;
    
 }

 DiffDrive::DiffDrive(double dist, double radius)
 {
   d = dist;
   r = radius;
   phi.right_phi = 0.0;
   phi.left_phi = 0.0;
   q.phi = 0.0;
   q.x = 0.0;
   q.y = 0.0;
    
 }

 DiffDrive::DiffDrive(double dist, double radius, WheelPhi phihat, Config qhat)
 {
   d = dist;
   r = radius;
   q = qhat;
   phi = phihat;

 }
 
 Config DiffDrive::ForwardKin(WheelPhi newphi)
 {
   Vector2D u;
   u.x = - phi.left_phi + newphi.left_phi;
   u.y =  - phi.right_phi + newphi.right_phi;
   Twist V;

   phi.left_phi = newphi.left_phi;
   phi.right_phi = newphi.right_phi;

   V.thetadot = -r*u.x/(2.0*d) + r*u.y/(2.0*d);
   V.xdot = r*u.x/2.0 + r*u.y/2.0;
   V.ydot = 0;

   Transform2D Tbb_hat = integrate_twist(V);

   Vector2D v_hat = Tbb_hat.translation();
   double phi_hat = Tbb_hat.rotation();

   Vector2D v;
   v.x = q.x;
   v.y = q.y;
   double v_phi = q.phi;
   Transform2D Twb(v, v_phi);

   Transform2D Twb_hat = Twb*Tbb_hat;
   Vector2D trans = Twb_hat.translation();
   double rot = Twb_hat.rotation();
   Config q_new;
   q_new.x = trans.x;
   q_new.y = trans.y;
   q_new.phi = rot;

   return q_new;
 }

 Vector2D DiffDrive::InvKin(Twist V)
 {
   if(!almost_equal(V.ydot,0.0, 0.1))
   {
      throw std::logic_error("Invalid Twist!");
   }
   else{
   Vector2D u;

   u.x = (-d*V.thetadot + V.xdot)/r;
   u.y =  (d*V.thetadot + V.xdot)/r;

   return u;
   }
 }

 Config DiffDrive::getConfig() const
{
    
   Config q_new;
   q_new.x = q.x;
   q_new.y = q.y;
   q_new.phi = q.phi;
   return q_new;
}

Twist DiffDrive::getTwist(WheelPhi newphi)
{
  Twist V;
  
  Vector2D u;
  u.x = - phi.left_phi + newphi.left_phi;
  u.y =  - phi.right_phi + newphi.right_phi;
  
  V.thetadot = -r*u.x/(2.0*d) + r*u.y/(2.0*d);
  V.xdot = r*u.x/2.0 + r*u.y/2.0;
  V.ydot = 0;

  // V.thetadot = (r/(2.0*d)*(- speed.left_phi + speed.right_phi));
  // V.xdot = r*(speed.left_phi + speed.right_phi)/2.0;
  // V.ydot = 0;

  return V; 

}

}
