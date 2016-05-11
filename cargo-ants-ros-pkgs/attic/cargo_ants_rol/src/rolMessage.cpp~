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
#include <iostream>


using namespace cargo_ants_udp;
  
  
  UDPMessage::UDPMessage (msg_int_t type_id, size_t n_ints, size_t n_reals)
  {
    resize (n_ints, n_reals);
    _typeID() = type_id;
  }
  
  
  void UDPMessage::
  resize (size_t n_ints, size_t n_reals)
  {
    buf_.resize ((3 + n_ints) * sizeof (msg_int_t) + n_reals * sizeof (msg_real_t), 0);
    _nInts() = n_ints;
    _nReals() = n_reals;
    std::cerr << "resize " << n_ints << " " << n_reals << " -- " << nInts() << " " << nReals() << "\n";
  }
  
  
  void UDPMessage::
  dump (ostream & os)
  {
    os << "type_id: " << typeID() << "\n"
       << "  n_ints: " << nInts() << "\n"
       << "  n_reals: " << nReals() << "\n";
    if (0 != nInts()) {
      os << "  ints:";
      for (int ii (0); ii < nInts(); ++ii) {
	os << "  " << ints()[ii];
      }
      os << "\n";
    }
    if (0 != nReals()) {
      os << "  reals:";
      for (int ii (0); ii < nReals(); ++ii) {
	os << "  " << reals()[ii];
      }
      os << "\n";
    }
  }
  

