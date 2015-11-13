/*
 * fpplib - Factory and Parameter Parsing Library
 *
 * Copyright (c) 2011 Roland Philippsen. All rights reserved.
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

#include <fpplib/factory.hpp>
#include <fpplib/configurable.hpp>
#include <iostream>


namespace fpplib {
  
  
  Factory::
  ~Factory()
  {
    for (creator_t::iterator ic(creator_.begin()); ic != creator_.end(); ++ic) {
      delete ic->second;
    }
  }
  
  
  Configurable * Factory::
  create(string const & type_name,
	 string const & instance_name)
  {
    creator_t::iterator ic(creator_.find(type_name));
    if (creator_.end() == ic) {
      return 0;
    }
    return ic->second->create(instance_name);
  }
  
  
  Configurable * Factory::
  find(string const & type_name,
       string const & instance_name)
    const
  {
    creator_t::const_iterator ic(creator_.find(type_name));
    if (creator_.end() != ic) {
      return ic->second->find(instance_name);
    }
    Configurable * const cc (findSingleton(type_name));
    if (cc && (instance_name == cc->name)) {
      return cc;
    }
    return 0;
  }
  
  
  Configurable * Factory::
  find(string const & instance_name)
    const
  {
    Configurable * cc (0);
    for (creator_t::const_iterator ic(creator_.begin()); ic != creator_.end(); ++ic) {
      cc = ic->second->find(instance_name);
      if (cc) {
	break;
      }
    }
    if ( ! cc) {
      for (singleton_t::const_iterator is(singleton_.begin()); is != singleton_.end(); ++is) {
	if (instance_name == is->second->name) {
	  cc = is->second;
	  break;
	}
      }
    }
    return cc;
  }
  
  
  Configurable * Factory::
  findSingleton(string const & type_name) const
  {
    singleton_t::const_iterator is(singleton_.find(type_name));
    if (singleton_.end() != is) {
      return is->second;
    }
    return 0;
  }
  
  
  void Factory::
  dump(string const & prefix, ostream & os)
    const
  {
    for (creator_t::const_iterator ic(creator_.begin()); ic != creator_.end(); ++ic) {
      os << prefix << ic->first << "\n";
      ic->second->dump (prefix + "  ", os);
    }
  }
  
}
