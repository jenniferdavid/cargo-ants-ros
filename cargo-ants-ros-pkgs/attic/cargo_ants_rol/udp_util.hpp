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

#ifndef CARGO_ANTS_UDP_UTIL_HPP
#define CARGO_ANTS_UDP_UTIL_HPP

#include <string>
#include <iosfwd>

extern "C" {
#include <sys/types.h>
#include <sys/socket.h>
}


namespace cargo_ants {

  using namespace std;
  
  
  // maybe later //  class UDPReceipt
  // maybe later //  {
  // maybe later //    friend class UDPServer;
  // maybe later //
  // maybe later //    UDPReceipt () : status_ (-2) {}
  // maybe later //    
  // maybe later //    struct sockaddr addr_;
  // maybe later //    socklen_t addr_len_;
  // maybe later //    int status_;
  // maybe later //    
  // maybe later //  public:
  // maybe later //    /**
  // maybe later //       Negative numbers are error codes, 0 or more means a number of
  // maybe later //       received bytes.
  // maybe later //    */
  // maybe later //    int status () const { return status_; }
  // maybe later //  };
  
  
  class UDPServer
  {
  public:
    UDPServer ();
    ~UDPServer ();
    
    int start (/** port specification, will get passed to getaddrinfo() */
	       string const & port,
	       ostream & erros);
    
    /** Make sure the buffer is big enough! */
    // maybe later //    UDPReceipt const
    int receive (void * buf, size_t buf_len);
    
    /** Make sure the buffer is big enough! */
    // maybe later //    UDPReceipt const
    int peek (void * buf, size_t buf_len);
    
    // maybe later // int send (void const * buf, size_t buf_len, UDPReceipt const & receipt);

  private:
    bool initialized_;
    int sockfd_;
  };
  
  
  class UDPClient
  {
  public:
    UDPClient ();
    ~UDPClient ();
    
    int init (/** Host to connect to. Gets passed to getaddrinfo(). */
	      string const & host,
	      /** Port to connect to. Gets passed to getaddrinfo(). */
	      string const & port,
	      ostream & erros);
    
    int write (void const * buf, size_t buf_len);
    
    // maybe later // int read (void * buf, size_t buf_len);

  private:
    bool initialized_;
    int sockfd_;
  };
  
}

#endif // CARGO_ANTS_UDP_UTIL_HPP
