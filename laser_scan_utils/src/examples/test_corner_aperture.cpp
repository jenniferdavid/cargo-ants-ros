//std includes
#include <iostream>

//laserscanutils
#include "entities.h"
#include "corner_detector.h"

//main
int main(int argc, char** argv)
{
    std::cout << "\n ========= Corner aperture test ===========\n";

    //declare two points and a corner in between
    Eigen::Vector3s p1; p1 << 2,4,1;
    Eigen::Vector3s p2; p2 << 4,1,1;
    Eigen::Vector3s c; c << 4,4,1;
    
    //compute aperture 
    laserscanutils::ScalarT ap = laserscanutils::cornerAperture(p1,c,p2);
    std::cout << "Aperture: " << ap << std::endl;
    
    //exit
    return 0;
}
