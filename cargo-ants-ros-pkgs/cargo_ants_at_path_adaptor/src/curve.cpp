#include "ros/ros.h"
#include "cargo_ants_msgs/ReferenceTrajectory.h"
#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <errno.h>
#include <err.h>
#include <iostream>
#include <unistd.h>


using namespace std;
//using namespace cargo_ants_rol;
using namespace cargo_ants_msgs;


int main (int argc, char ** argv)
{  
  ros::init (argc, argv, "curve");
  ros::NodeHandle node;
  ros::Publisher pub (node.advertise<ReferenceTrajectory> ("test_node_curve", 100));

  ros::Rate idle_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    
    ReferenceTrajectoryPoint pp;
    ReferenceTrajectory trj;
    
    for (int ii ; ii < 42; ++ii) {
            double y [42];
            int a = 2;
            double x[42] = {1,1,1,1,1,1,1,1,1,1,1,1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,20,20,20,20,20,20,20,20,20,20};
            y[ii] = (x[ii]) * (x[ii]);
            pp.xx   = x [ii];
            pp.yy   = y [ii];
            pp.th   = 0;
            pp.xd   = 0;
            pp.yd   = 0;
            pp.thd  = 0;
            pp.xdd  = 0;
            pp.ydd  = 0;
            pp.thdd = 0;
            trj.points.push_back (pp);
    }
    pub.publish (trj);
    cout << "published\n";
        
    }
}
