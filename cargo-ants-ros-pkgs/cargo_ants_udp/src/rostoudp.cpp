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
//#include "agv_odom_sim/agv_odom_sim_node.h"
#include <nav_msgs/Odometry.h>

using namespace cargo_ants_msgs;
using namespace cargo_ants_udp;
using namespace std;

static UDPClient client;

static void reftrj_cb (ReferenceTrajectory::ConstPtr const & rosmsg)

{
    cout << "we got trajectory message from ROS\n";
    
    UDPMessage udpmsg (1 + 9 * rosmsg->points.size());
    //  udpmsg.ints()[0]  = rosmsg->start.sec;
    // udpmsg.ints()[1]  = rosmsg->start.nsec;
   // udpmsg.doubles()[0] = rosmsg->dt;
    for (size_t ii (0); ii < rosmsg->points.size(); ++ii) {
        udpmsg.doubles()[ii * 9 + 1] = rosmsg->points[ii].xx;
        udpmsg.doubles()[ii * 9 + 2] = rosmsg->points[ii].yy;
        udpmsg.doubles()[ii * 9 + 3] = rosmsg->points[ii].th;
        udpmsg.doubles()[ii * 9 + 4] = rosmsg->points[ii].xd;
        udpmsg.doubles()[ii * 9 + 5] = rosmsg->points[ii].yd;
        udpmsg.doubles()[ii * 9 + 6] = rosmsg->points[ii].thd;
        udpmsg.doubles()[ii * 9 + 7] = rosmsg->points[ii].xdd;
        udpmsg.doubles()[ii * 9 + 8] = rosmsg->points[ii].ydd;
        udpmsg.doubles()[ii * 9 + 9] = rosmsg->points[ii].thdd;
    }
    
      if (udpmsg.buflen() == client.write (udpmsg.buf(), udpmsg.buflen())) {
        cout << "wrote\n";
        udpmsg.dump (cout);
    }
    else {
        cout << "failed to write\n";
    }
    
}

static void odom_cb (nav_msgs::Odometry::ConstPtr const & odom_msg)

{
    cout << "we got odom message from ROS\n";
    
    UDPMessage udpmsg1 (1 + 3 * 3);
    //  udpmsg.ints()[0]  = rosmsg->start.sec;
    // udpmsg.ints()[1]  = rosmsg->start.nsec;
    // udpmsg.doubles()[0] = rosmsg->dt;
    for (size_t ii (0); ii < 3; ++ii) {
        udpmsg1.doubles()[ii * 3 + 1] = odom_msg->pose.pose.position.x;
        udpmsg1.doubles()[ii * 3 + 2] = odom_msg->pose.pose.position.y;
        udpmsg1.doubles()[ii * 3 + 3] = odom_msg->pose.pose.position.z;
    }
    
    if (udpmsg1.buflen() == client.write (udpmsg1.buf(), udpmsg1.buflen())) {
        cout << "wrote\n";
        udpmsg1.dump (cout);
    }
    else {
        cout << "failed to write\n";
    }
    
}



int main(int argc, char **argv)
{
    if (argc < 3) {
        errx (EXIT_FAILURE, "specify where to send UDP messages (host and port)");
    }
    
    cout << "creating client for host " << argv[1] << " port " << argv[2] << "\n";
    if (0 != client.init (argv[1], argv[2], cerr)) {
        exit (EXIT_FAILURE);
    }
       
    ros::init (argc, argv, "ros2udp");
    ros::NodeHandle node;
    ros::Subscriber trajros2udp (node.subscribe ("trajectory", 1000, reftrj_cb));
    ros::Subscriber odomros2udp (node.subscribe ("/agv_odom_sim/odom", 1000, odom_cb));
    
  
    while (ros::ok()) {
    ros::spinOnce();
    cout << "." << flush;
    usleep (2000000);
    }
       
    return 0;
}
