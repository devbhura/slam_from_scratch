#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../include/turtlelib/rigid2d.hpp"

#include <cmath>

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