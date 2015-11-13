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

#ifndef FPPLIB_REFLECTABLE_HPP
#define FPPLIB_REFLECTABLE_HPP

#include <string>
#include <iosfwd>


namespace fpplib {
  
  using std::string;
  using std::ostream;
  
  
  /**
     Base class for things that can be reflected, such as named
     parameters that can be read and written via a superclass pointer
     to Configurable.
  */
  class Reflectable
  {
  public:
    /**
       Type name for this reflectable instance.
    */
    string const type;
    
    /**
       Instance name of this reflectable object.
    */
    string const name;
    
    Reflectable(string const & type_,
		string const & name_)
      : type(type_),
	name(name_)
    {}
    
    virtual ~Reflectable() {}
    
    /**
       Hook for debug methods that just print this reflectable to the
       provided ostream.
    */
    virtual void dump(string const & prefix, ostream & os) const = 0;
  };
  
}

#endif // FPPLIB_REFLECTABLE_HPP
