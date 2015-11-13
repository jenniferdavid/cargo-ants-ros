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

#ifndef FPPLIB_SLOT_HPP
#define FPPLIB_SLOT_HPP

#include <fpplib/reflectable.hpp>


namespace fpplib {
  
  class Configurable;
  
  
  class BaseSlot
    : public Reflectable
  {
  public:
    BaseSlot(string const & type,
	     string const & name)
      : Reflectable(type, name)
    {}
    
    virtual bool set(Configurable * instance) = 0;
  };
  
  
  template<typename Type>
  class Slot
    : public BaseSlot
  {
  public:
    Slot(string const & name,
	 Type ** slot)
      : BaseSlot(typeid(Type).name(), name),
	slot_(slot)
    {
    }
    
    virtual bool set(Configurable * instance)
    {
      if (0 == this) {
	return false;
      }
      Type * tt(dynamic_cast<Type*>(instance));
      if (0 == tt) {
	return false;
      }
      *slot_ = tt;
      return true;
    }
    
    virtual void dump(string const & prefix, ostream & os) const
    {
      os << prefix << name << " : " << type << " at " << *slot_ << "\n";
      if (0 != *slot_) {
	(*slot_)->dump(prefix + "  ", os);
      }
    }
    
  protected:
    Type ** slot_;
  };
  
  
  template<typename Type>
  class VectorSlot
    : public BaseSlot
  {
  public:
    VectorSlot(string const & name,
	       vector<Type *> * instvec)
      : BaseSlot(typeid(Type).name(), name),
	instvec_(instvec)
    {
    }
    
    virtual bool set(Configurable * instance)
    {
      if (0 == this) {
	return false;
      }
      Type * tt(dynamic_cast<Type*>(instance));
      if (0 == tt) {
	return false;
      }
      instvec_->push_back(tt);
      return true;
    }
    
    virtual void dump(string const & prefix, ostream & os) const
    {
      os << prefix << name << " : " << type << " of size " << instvec_->size() << "\n";
      for (size_t ii(0); ii < instvec_->size(); ++ii) {
	instvec_->at(ii)->dump(prefix + "  ", os);
      }
    }
    
  protected:
    vector<Type *> * instvec_;
  };
  
}

#endif // FPPLIB_SLOT_HPP
