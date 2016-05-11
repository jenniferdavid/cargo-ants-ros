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

#include "cargo_ants_rol/rolMessage.hpp"
#include "cargo_ants_rol/rolServer.hpp"
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
using namespace cargo_ants_rol;
using namespace cargo_ants_msgs;


int main (int argc, char ** argv)
{
  if (argc < 2) {
    errx (EXIT_FAILURE, "specify rol port number");
  }
  
  rolServer server;
  cout << "creating rol server on port " << argv[1] << "\n";
  if (0 != server.start (argv[1], cerr)) {
    exit (EXIT_FAILURE);
  }
  
  ros::init (argc, argv, "test_node_rol2ros");
  ros::NodeHandle node;
  ros::Publisher pub (node.advertise<ReferenceTrajectory> ("testrol2ros", 100));

  ros::Rate idle_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    
    rolMessage rolmsg (42, 0, 0);
    if (rolmsg.buflen() != server.peek (rolmsg.buf(), rolmsg.buflen())) {
      cout << "failed to peek\n";
      usleep (1000000);
      continue;
    }
    rolmsg.resize (rolmsg.nInts(), rolmsg.nReals());
    if (rolmsg.buflen() != server.receive (rolmsg.buf(), rolmsg.buflen())) {
      cout << "failed to receive (that's weird: we were able to peek!)\n";
      usleep (1000000);
      continue;
    }
    cout << "\nreceived\n";
    rolmsg.dump (cout);

    if ((rolmsg.nReals() < 1) || (rolmsg.nInts() < 2)) {
      cout << "invalid message: expected at least 1 real and 2 ints\n";
      usleep (1000000);
      continue;
    }
    
    ReferenceTrajectory trj;
  //  trj.start.sec = rolmsg.ints()[0];
  //  trj.start.nsec = rolmsg.ints()[1];
    trj.dt = rolmsg.reals()[0];
    for (int ii (1 + 8); ii < rolmsg.nReals(); ++ii) {
      ReferenceTrajectoryPoint pp;
      pp.xx   = rolmsg.reals()[ii - 8];
      pp.yy   = rolmsg.reals()[ii - 7];
      pp.th   = rolmsg.reals()[ii - 6];
      pp.xd   = rolmsg.reals()[ii - 5];
      pp.yd   = rolmsg.reals()[ii - 4];
      pp.thd  = rolmsg.reals()[ii - 3];
      pp.xdd  = rolmsg.reals()[ii - 2];
      pp.ydd  = rolmsg.reals()[ii - 1];
      pp.thdd = rolmsg.reals()[ii];
      trj.points.push_back (pp);
    }
    pub.publish (trj);
    cout << "published\n";
  }
}
