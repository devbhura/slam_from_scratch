#include "catch_ros/catch.hpp"
#include "../include/turtlelib/rigid2d.hpp"
#include "../include/turtlelib/diff_drive.hpp"
#include <sstream>
#include <cmath>



TEST_CASE("Forward kinematics Robot driving forward", "[DiffDrive]") { //Marco Morales
    turtlelib::Config config,new_config;
    turtlelib::WheelPhi phi,phi_new,phiold;
    double PI = turtlelib::PI;
    
    double d,r;
    d = 0.08;
    r = 0.033;
    turtlelib::DiffDrive diff = turtlelib::DiffDrive(d,r);

    phi_new.left_phi = PI/4;
    phi_new.right_phi = PI/4;

    new_config = diff.ForwardKin(phi_new);
    CHECK(new_config.x == Approx(0.033*(PI/4))); 
    CHECK(new_config.y == Approx(0));
    CHECK(new_config.phi == Approx(0)); 
}

TEST_CASE("Forward kinematics Robot driving rotation", "[DiffDrive]") { //Marco Morales
    turtlelib::Config config,new_config;
    turtlelib::WheelPhi phi,phi_new,phiold;
    double PI = turtlelib::PI;
    
    double d,r;
    d = 0.08;
    r = 0.033;
    turtlelib::DiffDrive diff = turtlelib::DiffDrive(d,r);

    phi_new.left_phi = PI/4;
    phi_new.right_phi = -PI/4;

    new_config = diff.ForwardKin(phi_new);
    CHECK(new_config.x == Approx(0.0)); 
    CHECK(new_config.y == Approx(0));
    CHECK(new_config.phi == Approx(-0.3239767424)); 
}



TEST_CASE( "Test for rotation", "rotation" ) {// Devesh Bhura

    turtlelib::Transform2D Tab;

    double theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta_check,0.0,0.01));
}

TEST_CASE( "Test for translation", "translation" ) {// Devesh Bhura

    turtlelib::Transform2D Tab;

    turtlelib::Vector2D v0 = Tab.translation();

    REQUIRE(turtlelib::almost_equal(v0.x,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,0.0,0.01));
}


TEST_CASE( "Test for rotation transform", "rotation transform" ) {// Devesh Bhura

    double theta = turtlelib::deg2rad(90);
    turtlelib::Vector2D v, v0;
    turtlelib::Transform2D Tab(theta);

    double theta_check;
    v0 = Tab.translation();
    theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta,theta_check,0.01));
    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.01));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.01));
}

TEST_CASE( "Test for translation transform", "translation transform" ) {// Devesh Bhura

    turtlelib::Vector2D v, v0;
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v);

    v0 = Tab.translation();
    double theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta_check,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.01));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.01));
}

TEST_CASE( "Test for rotation and translation transform", "rot & trans transform" ) {// Devesh Bhura

    turtlelib::Vector2D v, v0;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v, theta);

    v0 = Tab.translation();
    double theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta_check,theta,0.01));
    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.01));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.01));
}


TEST_CASE( "Test for Identity", "Identity" ) {// Devesh Bhura

    turtlelib::Vector2D v0;
    turtlelib::Transform2D Tab;

    v0 = Tab.translation();
    double theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta_check,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.x,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,0.0,0.01));
}

TEST_CASE( "Test for inv", "[inverse]" ) {// Devesh Bhura

    double theta = turtlelib::deg2rad(90);
    turtlelib::Vector2D v, v0;
    v.x = 0;
    v.y = 0;
    turtlelib::Transform2D Tab(v,theta), Tba;

    v0 = Tab(v);
    

    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.1));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.1));
}


TEST_CASE( "Test for vector transformation", "Vector Transform" ) {// Devesh Bhura

    turtlelib::Vector2D v, v0, vt;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    vt.x = 0.0;
    vt.y = 0.0;
    v0 = Tab(vt);

    REQUIRE(turtlelib::almost_equal(v0.x,v.x,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,v.y,0.01));
}

TEST_CASE( "Test for Twist transformation", "Twist Transform" ) {// Devesh Bhura

    turtlelib::Twist V0, Vt;
    turtlelib::Vector2D v;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    Vt.xdot = 0.0;
    Vt.ydot = 0.0;
    Vt.thetadot = 0.0;
    V0 = Tab(Vt);

    REQUIRE(turtlelib::almost_equal(V0.xdot,Vt.xdot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.ydot,Vt.ydot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.thetadot,Vt.thetadot,0.01));
}


TEST_CASE( "Test for Multiplication", "Multiplication" ) {// Devesh Bhura

    turtlelib::Vector2D v, v0, vt;
    double theta0;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    vt.x = 0.0;
    vt.y = 0.0;
    turtlelib::Transform2D Tab(v,theta), Tbc(vt,0);

    Tab*=Tbc;

    v0 = Tab.translation();
    theta0 = Tab.rotation();
    

    REQUIRE(turtlelib::almost_equal(theta0, theta,0.01));
    REQUIRE(turtlelib::almost_equal(v0.x, v.x,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y, v.y,0.01));
}

TEST_CASE("istream Vector input","[Vector2D]"){// James Avtges
    std::stringstream bracket;
    turtlelib::Vector2D bracketV;
    bracket.str("[1 1]");
    std::stringstream number;
    turtlelib::Vector2D numberV;
    number.str("1 1");
    bracket >> bracketV;
    number >> numberV;
    REQUIRE(bracketV.x == 1);
    REQUIRE(numberV.x == 1);
}

TEST_CASE("ostream Vector output","[Vector2D]"){// James Avtges
    std::stringstream vectorOut;
    turtlelib::Vector2D vector;
    vector.x = 9;
    vector.y = 1;

    vectorOut << vector;

    REQUIRE(vectorOut.str() == "[9, 1]\n");
}

TEST_CASE("istream Transform input","[Transform2D]"){ // James Avtges
    std::stringstream transform;
    turtlelib::Transform2D T(0);
    transform.str("deg: 80 x: 2 y: 4\n");
    transform >> T;

    turtlelib::Vector2D translation = T.translation();
    double rotation = turtlelib::rad2deg(T.rotation());
    REQUIRE(translation.x == 2);
    REQUIRE(translation.y == 4);
    REQUIRE(rotation == 80);
}

TEST_CASE("ostream Transform output","[Transform2D]"){// James Avtges
    std::stringstream transformOut;
    turtlelib::Vector2D trans;
    trans.x = 3.2;
    trans.y = 4;
    double rot = turtlelib::deg2rad(6.1);
    turtlelib::Transform2D T(trans,rot);

    transformOut << T;

    REQUIRE(transformOut.str() == "deg: 6.1 x: 3.2 y: 4\n");
}

TEST_CASE("Normalize Angle","[Normalize angle]"){// Devesh Bhura
    
    double norm_ang;
    double PI = turtlelib::PI;
    
    norm_ang = turtlelib::normalize_angle(PI);

    REQUIRE(turtlelib::almost_equal(norm_ang, PI, 0.01));

    norm_ang = turtlelib::normalize_angle(-PI);
    REQUIRE(turtlelib::almost_equal(norm_ang, PI, 0.01));

    norm_ang = turtlelib::normalize_angle(0);
    REQUIRE(turtlelib::almost_equal(norm_ang, 0, 0.01));

    norm_ang = turtlelib::normalize_angle(-PI/4);
    REQUIRE(turtlelib::almost_equal(norm_ang, -PI/4, 0.01));

    norm_ang = turtlelib::normalize_angle(3*PI/2);
    REQUIRE(turtlelib::almost_equal(norm_ang, -PI/2, 0.01));

    norm_ang = turtlelib::normalize_angle(-5*PI/2);
    REQUIRE(turtlelib::almost_equal(norm_ang, -PI/2, 0.01));

}

TEST_CASE("Integrate Twist (pure translational)","[Integrate Twist]"){// Devesh Bhura
    
    turtlelib::Twist V;
    V.xdot = 1;
    V.ydot = 1;

    turtlelib::Transform2D T;
    T = turtlelib::integrate_twist(V);
    turtlelib::Vector2D v0;
    v0 = T.translation();
    double ang = T.rotation();

    REQUIRE(turtlelib::almost_equal(v0.x, V.xdot, 0.01));
    REQUIRE(turtlelib::almost_equal(v0.y, V.ydot, 0.01));
    REQUIRE(turtlelib::almost_equal(ang, 0.0, 0.01));

}

TEST_CASE("Integrate Twist (pure rotational)","[Integrate Twist]"){// Devesh Bhura
    
    turtlelib::Twist V;
    V.thetadot = 1;

    turtlelib::Transform2D T;
    T = turtlelib::integrate_twist(V);
    turtlelib::Vector2D v0;
    v0 = T.translation();
    double ang = T.rotation();

    REQUIRE(turtlelib::almost_equal(v0.x, 0.0, 0.01));
    REQUIRE(turtlelib::almost_equal(v0.y, 0.0, 0.01));
    REQUIRE(turtlelib::almost_equal(ang, V.thetadot, 0.01));

}

TEST_CASE("Integrate Twist (both translational and rotational)","[Integrate Twist]"){// Devesh Bhura
    
    turtlelib::Twist V;
    V.xdot = 1;
    V.ydot = 1;
    double PI = turtlelib::PI;
    V.thetadot = PI;

    turtlelib::Transform2D T;
    T = turtlelib::integrate_twist(V);
    turtlelib::Vector2D v0;
    v0 = T.translation();
    double ang = T.rotation();

    CHECK(v0.x == Approx(-0.6366197724));
    CHECK(v0.y == Approx(0.6366197724));
    CHECK(ang == Approx(PI));

}

TEST_CASE("DiffDrive (Linear Drive forward)","[Linear Forward Kinematics]"){// Devesh Bhura
    
    turtlelib::WheelPhi newphi, phi;
    turtlelib::Config q, qnew;

    double d,r;
    d = 0.08;
    r = 0.033;
    phi.left_phi = 0.0;
    phi.right_phi = 0.0;
    q.x = 0.0;
    q.phi = 0.0;
    q.y = 0.0;

    turtlelib::DiffDrive tur(d, r, phi, q);

    newphi.right_phi = 0.2;
    newphi.left_phi = 0.2;

    qnew = tur.ForwardKin(newphi);


    REQUIRE(turtlelib::almost_equal(qnew.x, 0.0066, 0.0001));
    REQUIRE(turtlelib::almost_equal(qnew.y, 0.0000, 0.0001));
    REQUIRE(turtlelib::almost_equal(qnew.phi, 0.0000, 0.0001));

}

TEST_CASE("DiffDrive (Linear Drive forward from Twist)","[Linear Inverse Kinematics]"){// Devesh Bhura
    
    turtlelib::WheelPhi phi;
    turtlelib::Config q;
    turtlelib::Twist V;
    turtlelib::Vector2D v;


    double d,r;
    d = 0.08;
    r = 0.033;

    turtlelib::DiffDrive tur(d, r, phi, q);

    V.xdot = 1.0;

    v = tur.InvKin(V);


    CHECK(v.x == Approx(30.303030303));
    CHECK(v.y == Approx(30.303030303));

}

TEST_CASE("DiffDrive (Rot Drive forward)","[Rotational Forward Kinematics]"){// Devesh Bhura
    
    turtlelib::WheelPhi newphi, phi;
    turtlelib::Config q, qnew;

    double d,r;
    d = 0.08;
    r = 0.033;
    

    turtlelib::DiffDrive tur(d, r, phi, q);

    newphi.right_phi = 0.2;
    newphi.left_phi = 0;

    qnew = tur.ForwardKin(newphi);


    REQUIRE(turtlelib::almost_equal(qnew.x, 0.0033, 0.0001));
    REQUIRE(turtlelib::almost_equal(qnew.y, 0.0000, 0.0001));
    REQUIRE(turtlelib::almost_equal(qnew.phi, 0.04125, 0.0001));

}

TEST_CASE("DiffDrive (Rot Drive forward from Twist)","[Rotational Inverse Kinematics]"){// Devesh Bhura
    
    turtlelib::WheelPhi phi;
    turtlelib::Config q;
    turtlelib::Twist V;
    turtlelib::Vector2D v;


    double d,r;
    d = 0.08;
    r = 0.033;

    turtlelib::DiffDrive tur(d, r, phi, q);

    V.thetadot = 1.0;

    v = tur.InvKin(V);


    REQUIRE(turtlelib::almost_equal(v.x, -2.42, 0.01));
    REQUIRE(turtlelib::almost_equal(v.y, 2.42, 0.01));
    
}

TEST_CASE("DiffDrive (Rot Drive Circle)","[Circle Forward Kinematics]"){// Devesh Bhura
    
    turtlelib::WheelPhi phi;
    turtlelib::Config q;
    turtlelib::Twist V;
    turtlelib::Vector2D v;


    double d,r;
    d = 0.08;
    r = 0.033;

    turtlelib::DiffDrive tur(d, r, phi, q);

    V.thetadot = 1.0;
    V.xdot = 1.0;

    v = tur.InvKin(V);


    CHECK(v.x == Approx(27.8787878788) );
    CHECK(v.y == Approx(32.7272727273) );
    
}

TEST_CASE("DiffDrive (Rot Drive Circle from Twist)","[Circle Inverse Kinematics]"){// Devesh Bhura
    
    turtlelib::WheelPhi newphi, phi;
    turtlelib::Config q, qnew;

    double d,r;
    d = 0.08;
    r = 0.033;

    turtlelib::DiffDrive tur(d, r, phi, q);

    newphi.right_phi = 0.2;
    newphi.left_phi = -0.1;

    qnew = tur.ForwardKin(newphi);

    REQUIRE(turtlelib::almost_equal(qnew.x, 0.00165, 0.0001));
    REQUIRE(turtlelib::almost_equal(qnew.y, 0.00, 0.01));
    REQUIRE(turtlelib::almost_equal(qnew.phi, 0.06187, 0.0001));
    
}

TEST_CASE("DiffDrive (Invalid Twist)","[Invalid Twist]"){// Devesh Bhura
    
    turtlelib::WheelPhi phi;
    turtlelib::Config q;
    turtlelib::Twist V;
    turtlelib::Vector2D v;


    double d,r;
    d = 0.08;
    r = 0.033;

    turtlelib::DiffDrive tur(d, r, phi, q);

    V.thetadot = 1.0;
    V.xdot = 1.0;
    V.ydot = 2.0;

    CHECK_THROWS(v = tur.InvKin(V));
    
}