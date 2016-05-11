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

#ifndef CARGO_ANTS_UDP_MESSAGE_HPP
#define CARGO_ANTS_UDP_MESSAGE_HPP

#include <vector>
#include <sstream>
#include <string>



namespace cargo_ants_udp {
  
  using namespace std;
  
  typedef int msg_int_t;
  typedef double msg_double_t;
  
  
  class UDPMessage
  {
  public:
    UDPMessage (cargo_ants_udp::size_t n_doubles);
    
     void resize (size_t n_doubles);

  private:
    //  msg_int_t & _typeID () { return *reinterpret_cast<msg_int_t*>(&buf_[                   0]); }
    msg_int_t & _nInts ()  { return *reinterpret_cast<msg_int_t*>(&buf_[  sizeof (msg_int_t)]); }
    msg_int_t & _nDoubles () { return *reinterpret_cast<msg_int_t*>(&buf_[2*sizeof (msg_int_t)]); }
    
  public:
    // msg_int_t const & typeID () const { return const_cast<UDPMessage*>(this)->_typeID(); }
   msg_int_t const & nInts () const  { return const_cast<UDPMessage*>(this)->_nInts(); }
   msg_int_t const & nDoubles () const { return const_cast<UDPMessage*>(this)->_nDoubles(); }
    
    // msg_int_t * ints ()   { return reinterpret_cast<msg_int_t*>(&buf_[3 * sizeof (msg_int_t)]); }
    msg_double_t * doubles () { return reinterpret_cast<msg_double_t*>(&buf_[(3 + nInts()) * sizeof (msg_int_t)]); }
    
    void * buf ()
    { return &buf_[0]; }
    
    int buflen ()
    { return buf_.size(); }
    
    void dump (ostream & os); 
    
   private:
    vector <int> buf_;
  };
  
}

#endif // CARGO_ANTS_UDP_MESSAGE_HPP
