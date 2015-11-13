/*
 * fpplib - Factory and Parameter Parsing Library
 *
 * Copyright (c) 2012 Roland Philippsen. All rights reserved.
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

#ifndef FPPLIB_CALLBACK_HPP
#define FPPLIB_CALLBACK_HPP

#include <fpplib/reflectable.hpp>


namespace fpplib {
  
  
  class BaseCallback
    : public Reflectable
  {
  public:
    BaseCallback(string const & type,
		 string const & name,
		 bool sequence_mode_,
		 BaseParameter * argptr)
      : Reflectable(type, name),
	sequence_mode(sequence_mode_),
	argptr_(argptr)
    {}
    
    ~BaseCallback()
    {
      delete argptr_;
    }
    
    virtual bool call (ostream & erros) = 0;
    
    bool const sequence_mode;
    BaseParameter * argptr_;
  };
  
  
  template<typename value_type, typename callable_type>
  class Callback
    : public BaseCallback
  {
  public:
    Callback(string const & name,
	     bool sequence_mode,
	     callable_type callback)
      : BaseCallback(typeid(value_type).name(), name, sequence_mode,
		     new Parameter<value_type>(name, &arginst, 0)),
	callback_(callback)
    {
    }
    
    virtual bool call (ostream & erros)
    {
      if (0 == this) {
	erros << "callback `" << name << "' is null\n";
	return false;
      }
      return callback_(arginst, erros);
    }
    
    virtual void dump(string const & prefix, ostream & os) const
    {
      os << prefix << name << " : callback for " << type << "\n";
    }
    
    value_type arginst;
    
  protected:
    callable_type callback_;
  };
  
}

#endif // FPPLIB_SLOT_HPP
