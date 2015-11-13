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

#ifndef FPPLIB_PARAMETER_HPP
#define FPPLIB_PARAMETER_HPP

#include <fpplib/reflectable.hpp>
#include <iostream>
#include <vector>
#include <typeinfo>


namespace fpplib {
  
  using std::ostream;
  using std::vector;
  
  
  /**
     Functor interface for optionally guarding a Parameter from being
     overwritten with an invalid value.
     
     \todo XXXX to do: parameter guards may need to be parameter type
     specific, for example in order to support vector parameters.
  */
  class ParameterGuard
  {
  public:
    string const description;
    
    explicit ParameterGuard (string const & description_)
      : description (description_)
    {}
    
    virtual ~ParameterGuard() {}
    
    /**
       Interface for checking whether a parameter instance can be
       overwritten with the given value. It is used by
       Parameter<>::set.
       
       \return True if it is okay to overwrite, false otherwise.
     */
    virtual bool check(void const * instance, void const * value) const = 0;
  };
  
  
  /**
     Functor for read-only Parameter instances.
  */
  class ReadOnlyGuard
    : public ParameterGuard
  {
  public:
    ReadOnlyGuard()
      : ParameterGuard ("read-only")
    {}
    
    virtual bool check(void const * instance, void const * value) const
    {
      return false;
    }
  };
  
  
  /**
     Make sure given values are strictly positive.
  */
  template <typename value_type>
  class StrictlyPositiveGuard
    : public ParameterGuard
  {
  public:
    StrictlyPositiveGuard()
      : ParameterGuard ("strictly-positive")
    {}
    
    virtual bool check(void const * instance, void const * value) const
    {
      if (0 >= * reinterpret_cast <value_type const *> (value)) {
	return false;
      }
      return true;
    }
  };
  
  
  /**
     Base class for all reflectable parameters. Normally, client code
     does not need to be aware of this class, nor indeed of its
     implementation via the Parameter<> template: use
     Confiburable::reflectParameter instead.
  */
  class BaseParameter
    : public Reflectable
  {
  public:
    /**
       \todo XXXX to do: who deletes the guard?
    */
    ParameterGuard const * guard;
    
    BaseParameter(string const & type,
		  string const & name,
		  ParameterGuard const * guard);
  };
  
  
  /**
     Generic Parameter wrapper to make (hopefully) any type of field
     reflectable.
  */
  template<typename value_type>
  class Parameter
    : public BaseParameter
  {
  public:
    Parameter(string const & name,
	      value_type * instance,
	      ParameterGuard const * guard)
      : BaseParameter(typeid(value_type).name(), name, guard),
	instance_(instance)
    {
    }
    
    value_type const * get() const
    {
      return instance_;
    }
    
    bool set(value_type const & value, ostream & erros)
    {
      if (0 == this) {
	erros << "BUG? null == this";
	return false;
      }
      if ((0 != guard)
	  && ( ! guard->check(instance_, &value))) {
	erros << "guard check failed: " << guard->description;
	return false;
      }
      *instance_ = value;
      return true;
    }
    
    virtual void dump(string const & prefix, ostream & os) const
    {
      os << prefix << name << " : " << type << " = " << *instance_ << "\n";
    }
    
  protected:
    value_type * instance_;
  };
  
  
  /**
     Generic VectorParameter wrapper to make (hopefully) any type of
     std::vector<> field reflectable.
     
     \todo XXXX to do: leaving out the parameter guard idea for now,
     this is still a rough sketch.
  */
  template<typename value_type>
  class VectorParameter
    : public BaseParameter
  {
  public:
    typedef typename std::vector<value_type> vector_type;
    
    VectorParameter(string const & name,
		    vector_type * instance)
      : BaseParameter(typeid(vector_type).name(), name, 0),
	instance_(instance)
    {
    }
    
    vector_type const * get() const
    {
      return instance_;
    }
    
    vector_type * get()
    {
      return instance_;
    }
    
    virtual void dump(string const & prefix, ostream & os) const
    {
      os << prefix << name << " : " << type;
      if (instance_->empty()) {
	os << " (empty)\n";
      }
      else {
	os << " (size " << instance_->size() << ")\n";
	for (size_t ii(0); ii < instance_->size(); ++ii) {
	  os << prefix << "  [" << ii << "] " << instance_->at(ii) << "\n";
	}
      }
    }
    
  protected:
    vector_type * instance_;
  };
  
}

#endif // FPPLIB_PARAMETER_HPP
