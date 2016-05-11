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

#include "cargo_ants_rol/rolServer.hpp"
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


using namespace cargo_ants_rol;

namespace {
  
  
  int create_rol_server (char const * port, int ai_family,
			 int & sockfd, ostream & erros)
  {
    struct addrinfo hints;
    memset (&hints, 0, sizeof (hints));
    hints.ai_family = ai_family;
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
    hints.ai_protocol = 0;          /* Any protocol */
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;
    
    struct addrinfo * result;
    int const status (getaddrinfo (NULL, port, &hints, &result));
    if (status != 0) {
      erros << "create_rol_server (" << port
	    << "): getaddrinfo: " << gai_strerror (status) << "\n";
      return -1;
    }
    
    struct addrinfo * rp;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
      sockfd = socket (rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (sockfd == -1) {
	continue;
      }
      if (bind (sockfd, rp->ai_addr, rp->ai_addrlen) == 0) {
	// bound to port
	break;
      }
      close (sockfd);
    }
    if (rp == NULL) {
      erros << "create_rol_server (" << port << "): could not bind\n";
      freeaddrinfo (result);
      return -2;
    }
    
    freeaddrinfo (result);
    return 0;
  }
  
  
  int rol_server_recvfrom (int sfd, void * buf, size_t buf_len, int flags,
  			   struct sockaddr * addr, socklen_t * addr_len)
  {
    int const nread (recvfrom (sfd, buf, buf_len, flags, addr, addr_len));
    if (-1 == nread) {
      // "rol_server_recvfrom(): recvfrom: " << strerror(errno) ...
      return -1;
    }
    return nread;
  }
  
}


namespace cargo_ants_rol {
  
  
  rolServer::
  rolServer ()
    : initialized_ (false)
  {
  }
  
  
  rolServer::
  ~rolServer()
  {
    if (initialized_) {
      close (sockfd_);
    }
  }
  
  
  int rolServer::
  start (string const & port,
	 ostream & erros)
  {
    if (initialized_) {
      erros << "rolServer::start (): already initialized\n";
      return -10;
    }
    
    static int const ai_family (AF_INET); // use AF_INET6 if you want IPv6
    int status (create_rol_server (port.c_str(), ai_family, sockfd_, erros));
    if (0 != status) {
      return status;
    }
    
    initialized_ = true;
    return 0;
  }
  
  
  int rolServer::
  receive (void * buf, size_t buf_len)
  {
    if ( ! initialized_ ) {
      return -10;
    }
    socklen_t addr_len;
    return rol_server_recvfrom (sockfd_, buf, buf_len, 0 /*flags*/, NULL, &addr_len);
  }
  
  
  int rolServer::
  peek (void * buf, size_t buf_len)
  {
    if ( ! initialized_ ) {
      return -10;
    }
    socklen_t addr_len;
    return rol_server_recvfrom (sockfd_, buf, buf_len, MSG_PEEK /*flags*/, NULL, &addr_len);
  }
  
}
