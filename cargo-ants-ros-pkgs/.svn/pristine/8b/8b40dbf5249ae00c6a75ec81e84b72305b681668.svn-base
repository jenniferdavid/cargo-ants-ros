

//std
#include <iostream>

//package
#include "../grid_2d.h"

int main(int argc, char** argv)
{
    std::cout << "\n ========= Odometry update test ===========\n";

    //create a grid
    Grid2D *grid = new Grid2D(10,10,0.5,0.5,10,5);
    grid->print();
    
    //fill it with some arbitrary values
    grid->debugFill();
    grid->display();
    
    //update with odometry and display again
	grid->odometry_.push_back(Eigen::Vector4d(3,2,1,1));
    grid->odometryUpdate();
    grid->display();    
    
    //delete grid
    delete grid;
    
    //return
    return 0;
}
