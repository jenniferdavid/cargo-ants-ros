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
       
    ros::init (argc, argv, "ros2udp_odom");
    ros::NodeHandle node;
    ros::Subscriber odomros2udp (node.subscribe ("/at_odom_sim/odom", 1000, odom_cb));
    
  
    while (ros::ok()) {
    ros::spinOnce();
    cout << "." << flush;
    usleep (2000000);
    }
       
    return 0;
}
