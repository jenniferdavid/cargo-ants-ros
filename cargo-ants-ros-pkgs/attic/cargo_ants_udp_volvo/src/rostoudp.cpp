#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include <iostream>
#include <stdlib.h>
#include <iostream>
#include "ros/ros.h"
#include "cargo_ants_msgs/ReferenceTrajectory.h"
#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <errno.h>
#include <err.h>
#include <unistd.h>
#include "cargo_ants_udp/UDPMessage.hpp"
#include "cargo_ants_udp/UDPClient.hpp"

using namespace cargo_ants_msgs;
using namespace cargo_ants_udp;
using namespace std;

static UDPClient client;

static void reftrj_cb (ReferenceTrajectory::ConstPtr const & rosmsg)

{
    cout << "we got a message from ROS\n";
    
    UDPMessage udpmsg (42, 2, 1 + 9 * rosmsg->points.size());
    //  udpmsg.ints()[0]  = rosmsg->start.sec;
    // udpmsg.ints()[1]  = rosmsg->start.nsec;
    udpmsg.reals()[0] = rosmsg->dt;
    for (size_t ii (0); ii < rosmsg->points.size(); ++ii) {
        udpmsg.reals()[ii * 9 + 1] = rosmsg->points[ii].xx;
        udpmsg.reals()[ii * 9 + 2] = rosmsg->points[ii].yy;
        udpmsg.reals()[ii * 9 + 3] = rosmsg->points[ii].th;
        udpmsg.reals()[ii * 9 + 4] = rosmsg->points[ii].xd;
        udpmsg.reals()[ii * 9 + 5] = rosmsg->points[ii].yd;
        udpmsg.reals()[ii * 9 + 6] = rosmsg->points[ii].thd;
        udpmsg.reals()[ii * 9 + 7] = rosmsg->points[ii].xdd;
        udpmsg.reals()[ii * 9 + 8] = rosmsg->points[ii].ydd;
        udpmsg.reals()[ii * 9 + 9] = rosmsg->points[ii].thdd;
    }
    
    if (udpmsg.buflen() == client.write (udpmsg.buf(), udpmsg.buflen())) {
        cout << "wrote\n";
        udpmsg.dump (cout);
    }
    else {
        cout << "failed to write\n";
    }
    
}

int main(int argc, char **argv)
{
   
    ros::init (argc, argv, "ros2udp");
    ros::NodeHandle node;
    ros::Subscriber ros2udp (node.subscribe ("trajectory", 1000, reftrj_cb));
  
    cout << "subscribed to udp, calling ros::spin\n";
    
    while (ros::ok()) {
    ros::spinOnce();
    cout << "." << flush;
    usleep (2000000);
    }
       
    return 0;
}
