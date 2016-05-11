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

#include "cargo_ants_udp/UDPMessage.hpp"
#include "cargo_ants_udp/UDPClient.hpp"
#include <iostream>
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


int main (int argc, char ** argv)
{
  if (argc < 3) {
    errx (EXIT_FAILURE, "specify where to send UDP messages (host and port)");
  }
  
  UDPClient client;
  cout << "creating client for host " << argv[1] << " port " << argv[2] << "\n";
  if (0 != client.init (argv[1], argv[2], cerr)) {
    exit (EXIT_FAILURE);
  }
  UDPMessage msg (42, 2, 10);
  for (size_t ii (0); true; ii = (ii + 1) % 100) {
    msg.ints()[0]  = 1000 + ii;
    msg.ints()[1]  = 2000 + ii;
    msg.reals()[0] = 0.1;
    for (size_t jj (1); jj < 10; ++jj) {
      msg.reals()[jj] = 100000 + 10 * ii + jj;
    }
    if (msg.buflen() == client.write (msg.buf(), msg.buflen())) {
      cout << "wrote\n";
      msg.dump (cout);
      usleep (500000);
    }
    else {
      cout << "failed to write\n";
      usleep (1000000);
    }
  }
}
