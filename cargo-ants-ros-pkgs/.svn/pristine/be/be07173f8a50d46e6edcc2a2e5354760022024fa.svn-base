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
#include "cargo_ants_rol/rolClient.hpp"
#include "cargo_ants_msgs/ReferenceTrajectory.h"
#include <unistd.h>

#include <err.h>
#include <iostream>

using namespace std;
using namespace cargo_ants_rol;
using namespace cargo_ants_msgs;

static rolClient client;


static void reftrj_cb (ReferenceTrajectory::ConstPtr const & rosmsg)
{
  cout << "we got a message from ROS\n";
  
  rolMessage rolmsg (42, 2, 1 + 9 * rosmsg->points.size());
  //rolmsg.ints()[0]  = rosmsg->start.sec;
 // rolmsg.ints()[1]  = rosmsg->start.nsec;
  rolmsg.reals()[0] = rosmsg->dt;
  for (size_t ii (0); ii < rosmsg->points.size(); ++ii) {
    rolmsg.reals()[ii * 9 + 1] = rosmsg->points[ii].xx;
    rolmsg.reals()[ii * 9 + 2] = rosmsg->points[ii].yy;
    rolmsg.reals()[ii * 9 + 3] = rosmsg->points[ii].th;
    rolmsg.reals()[ii * 9 + 4] = rosmsg->points[ii].xd;
    rolmsg.reals()[ii * 9 + 5] = rosmsg->points[ii].yd;
    rolmsg.reals()[ii * 9 + 6] = rosmsg->points[ii].thd;
    rolmsg.reals()[ii * 9 + 7] = rosmsg->points[ii].xdd;
    rolmsg.reals()[ii * 9 + 8] = rosmsg->points[ii].ydd;
    rolmsg.reals()[ii * 9 + 9] = rosmsg->points[ii].thdd;
  }
  
  if (rolmsg.buflen() == client.write (rolmsg.buf(), rolmsg.buflen())) {
    cout << "wrote\n";
    rolmsg.dump (cout);
  }
  else {
    cout << "failed to write\n";
  }

}
int main (int argc, char ** argv)
{
  if (argc < 3) {
    errx (EXIT_FAILURE, "specify where to send rol messages (host and port)");
  }
  
  cout << "creating client for host " << argv[1] << " port " << argv[2] << "\n";
  if (0 != client.init (argv[1], argv[2], cerr)) {
    exit (EXIT_FAILURE);
  }

  ros::init (argc, argv, "test_node_ros2rol");
  ros::NodeHandle node;
  ros::Subscriber blah (node.subscribe ("testros2rol", 100, reftrj_cb));
  
  cout << "subscribed to /testros2rol, calling ros::spin\n";
    
  while (ros::ok()) {
    ros::spinOnce();
    cout << "." << flush;
    usleep (2000);
  }
}
