/* Cargo-ANTs software prototype.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * Based on code examples from the getaddrinfo(3) Linux man page.
 *
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

#include "cargo_ants_udp/UDPClient.hpp"
#include <iostream>

extern "C" {
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include <errno.h>
#ifdef OPENBSD
# include <netinet/in.h>
# include <netinet/in_systm.h>
#endif // OPENBSD
#include <netinet/ip.h>
}


using namespace cargo_ants_udp;

namespace {
  
  
  int create_udp_client (char const * host, char const * port, int ai_family,
			 int & sockfd, ostream & erros)
  {
    struct addrinfo hints;
    memset (&hints, 0, sizeof(hints));
    hints.ai_family = ai_family;
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = 0;
    hints.ai_protocol = 0;          /* Any protocol */
    
    struct addrinfo * result;
    int const status (getaddrinfo(host, port, &hints, &result));
    if (status != 0) {
      erros << "create_udp_client (" << host << ", " << port
	    << "): getaddrinfo: " << gai_strerror(status) << "\n";
      return -1;
    }
    
    struct addrinfo * rp;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
      sockfd = socket (rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (sockfd == -1) {
	continue;
      }
      if (connect(sockfd, rp->ai_addr, rp->ai_addrlen) != -1) {
	// "create_udp_client(): connected to port " << port << " on " << host ...
	break;
      }
      close (sockfd);
    }
    if (rp == NULL) {
      erros << "create_udp_client(" << host << ", " << port
	    << "): getaddrinfo: could not connect\n";
      freeaddrinfo(result);
      return -2;
    }
    
    freeaddrinfo(result);
    return 0;
  }
  
  
  int udp_client_write(int cfd, void const * buf, size_t buf_len)
  {
    int const nwritten (write (cfd, buf, buf_len));
    if (-1 == nwritten) {
      // "udp_client_write(): write: " << strerror(errno) ...
      return -1;
    }
    
    return nwritten;
  }
  
}


namespace cargo_ants_udp {
  
  
  UDPClient::
  UDPClient()
    : initialized_ (false)
  {
  }

  
  UDPClient::
  ~UDPClient ()
  {
    if (initialized_) {
      close (sockfd_);
    }
  }
  
  
  int UDPClient::
  init (string const & host,
	string const & port,
	ostream & erros)
  {
    if (initialized_) {
      erros << "UDPClient::init (): already initialized\n";
      return -10;
    }
    
    static int const ai_family (AF_INET); // use AF_INET6 if you want IPv6
    int status (create_udp_client (host.c_str(), port.c_str(), ai_family, sockfd_, erros));
    if (0 != status) {
      return status;
    }
    
    initialized_ = true;
    return 0;
  }
  
  
  int UDPClient::
  write (void const * buf, size_t buf_len)
  {
    return udp_client_write (sockfd_, buf, buf_len);
  }
  
}
