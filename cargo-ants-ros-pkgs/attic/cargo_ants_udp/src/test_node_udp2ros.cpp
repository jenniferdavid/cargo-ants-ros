/* Cargo-ANTs software prototype.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"

#include "cargo_ants_udp/UDPMessage.hpp"
#include "cargo_ants_udp/UDPServer.hpp"
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
using namespace cargo_ants_udp;
using namespace cargo_ants_msgs;


int main (int argc, char ** argv)
{
  if (argc < 2) {
    errx (EXIT_FAILURE, "specify UDP port number");
  }
  
  UDPServer server;
  cout << "creating UDP server on port " << argv[1] << "\n";
  if (0 != server.start (argv[1], cerr)) {
    exit (EXIT_FAILURE);
  }
  
  ros::init (argc, argv, "test_node_udp2ros");
  ros::NodeHandle node;
  ros::Publisher pub (node.advertise<ReferenceTrajectory> ("testudp2ros", 100));

  ros::Rate idle_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    
    UDPMessage udpmsg (42, 0, 0);
    if (udpmsg.buflen() != server.peek (udpmsg.buf(), udpmsg.buflen())) {
      cout << "failed to peek\n";
      usleep (1000000);
      continue;
    }
    udpmsg.resize (udpmsg.nInts(), udpmsg.nReals());
    if (udpmsg.buflen() != server.receive (udpmsg.buf(), udpmsg.buflen())) {
      cout << "failed to receive (that's weird: we were able to peek!)\n";
      usleep (1000000);
      continue;
    }
    cout << "\nreceived\n";
    udpmsg.dump (cout);

    if ((udpmsg.nReals() < 1) || (udpmsg.nInts() < 2)) {
      cout << "invalid message: expected at least 1 real and 2 ints\n";
      usleep (1000000);
      continue;
    }
    
    ReferenceTrajectory trj;
    trj.start.sec = udpmsg.ints()[0];
    trj.start.nsec = udpmsg.ints()[1];
    trj.dt = udpmsg.reals()[0];
    for (int ii (1 + 8); ii < udpmsg.nReals(); ++ii) {
      ReferenceTrajectoryPoint pp;
      pp.xx   = udpmsg.reals()[ii - 8];
      pp.yy   = udpmsg.reals()[ii - 7];
      pp.th   = udpmsg.reals()[ii - 6];
      pp.xd   = udpmsg.reals()[ii - 5];
      pp.yd   = udpmsg.reals()[ii - 4];
      pp.thd  = udpmsg.reals()[ii - 3];
      pp.xdd  = udpmsg.reals()[ii - 2];
      pp.ydd  = udpmsg.reals()[ii - 1];
      pp.thdd = udpmsg.reals()[ii];
      trj.points.push_back (pp);
    }
    pub.publish (trj);
    cout << "published\n";
  }
}
