#include "rigid2d.hpp"
#include <iostream>

int main()
{
    turtlelib::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;

    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;


    std::cout <<"T_{a,b}: " << T_ab << std::endl;
    T_ba = T_ab.inv();
    std::cout <<"T_{b,a}: " << T_ba << std::endl;

    std::cout <<"T_{b,c}: " << T_bc << std::endl;
    T_cb = T_bc.inv();
    std::cout <<"T_{c,b}: " << T_cb << std::endl;

    T_ac = T_ab*T_bc;
    T_ca = T_ac.inv();

    std::cout <<"T_{a,c}: " << T_ac << std::endl;
    std::cout <<"T_{c,a}: " << T_ca << std::endl;
    

}