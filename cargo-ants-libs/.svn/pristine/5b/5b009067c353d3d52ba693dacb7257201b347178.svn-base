#include <iostream>
#include <Eigen/Dense>
#include "chomp.hpp"

int main()
{
  Eigen::VectorXd qs(2); //Start goal  coordinates (x,y)
  Eigen::VectorXd qe(2); //End goal  coordinates (x,y)
  Eigen::VectorXd xi; //Trajectory points (x0,y0,x1,y1,....)
  Eigen::MatrixXd obs; //obstacles |x0,y0,R0|
                      //           |x1,y1,R1|
                      //           | .......|
  qs<<0,0;
  qe<<3,5;

  chomp::generatePath(qs,qe,xi,obs);
  std::cout<<xi<<std::endl;
}
