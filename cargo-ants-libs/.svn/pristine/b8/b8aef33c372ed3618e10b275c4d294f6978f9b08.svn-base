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

#ifndef FPPLIB_FACTORY_HPP
#define FPPLIB_FACTORY_HPP

#include <fpplib/registry.hpp>
#include <fpplib/configurable.hpp>
#include <iosfwd>
#include <typeinfo>
#include <set>


namespace fpplib {
  
  using std::string;
  using std::ostream;
  
  
  /**
     Interface for the Creator objects that are used by Factory to
     create Configurable subclass instances based on their type
     name. Normally, it should not be necessary to derive your own
     custom classes from BaseCreator, or even the provided Creator<>
     template, because Factory::declare does all of the work for you..
   */
  class BaseCreator
  {
  public:
    virtual ~BaseCreator() {}
    virtual Configurable * create(string const & instance_name) = 0;
    virtual Configurable * find(string const & instance_name) = 0;
    virtual void dump(string const & prefix, ostream & os) const = 0;
  };
  
  
  /**
     Sub-type-specific Creator. It is used by the Factory to manage
     its dictionary of Configurable sub types. Normally, your code
     should not need to be aware of this template, just call
     Factory::declare instead.
  */
  template<class SubType>
  class Creator
    : public BaseCreator
  {
  public:
    virtual SubType * create(string const & instance_name)
    {
      SubType * instance(new SubType(instance_name));
      registry_.add(instance_name, instance);
      return instance;
    }
    
    virtual SubType * find(string const & instance_name)
    {
      return registry_.find(instance_name);
    }
    
    virtual void dump(string const & prefix, ostream & os) const
    {
      for (size_t ii(0); ii < registry_.size(); ++ii) {
	registry_.at(ii)->dump(prefix, os);
      }
    }
    
    typedef Registry<SubType> registry_t;
    registry_t registry_;
  };
  
  
  /**
     A Factory is an object that creates Configurable objects. You can
     register new types by calling Factory::declare, and then use
     Factory::create to allocate and construct new instances
     thereof. Due to the fact that every Creator maintains a registry
     of the instances that it created, you can also look up existing
     instances using Factory::find.
  */
  class Factory
  {
  public:
    virtual ~Factory();
    
    /**
       Register a new type of Configurable with the Factory, so that
       later it can be created using the type_name (thus, clients need
       not be aware of the subclass implementation, nor indeed of its
       class name).
       
       \note If the given type_name is already in use, the old
       declaration will be discarded and henceworth instances of the
       latest registered SubType will be returned by Factory::create.

       \todo Is it wise to silently replace previously existing type
       declarations?  Exceptions might come in handy here, but that
       may open up a can of worms...
    */
    template<class SubType>
    void declare(string const & type_name)
    {
      creator_t::iterator ic(creator_.find(type_name));
      if (ic == creator_.end()) {
	creator_.insert(make_pair(type_name, new Creator<SubType>()));
	type_code_to_name_.insert(make_pair(string(typeid(SubType).name()), type_name));
      }
      else {
	delete ic->second;
	ic->second = new Creator<SubType>();
      }
    }
    
    template<class SubType>
    void declareSingleton(string const & type_name, SubType * instance)
    {
      singleton_t::iterator is(singleton_.find(type_name));
      if (is == singleton_.end()) {
	singleton_.insert(make_pair(type_name, instance));
	type_code_to_name_.insert(make_pair(string(typeid(SubType).name()), type_name));
      }
      else {
	is->second = instance;
      }
    }
    
    /**
       Create a new instance of a Configurable subclass, as previously
       registered using Factory::declare.
       
       \return A freshly constructed instance, or zero if the given
       type_name was not previously registered.
    */
    Configurable * create(string const & type_name,
			  string const & instance_name);

    /**
       Find an existing Configurable instance, given its type and
       instance names. See also the templatized find<> method.  It
       first searches through the non-singleton configurables. If that
       fails, it checks if there is a singleton with matching
       instance_name for that type_name.
       
       \return The existing instance, or zero if either the type or
       instance name did not match.

       \todo XXXX: kick this out, it is too cumbersome to need a type
       name for looking up instances. Also, take the SubType registry
       out of the Creators, just keep a single registry of
       configurables here in the factory, and hook instances into it
       before returning from create().
    */    
    Configurable * find(string const & type_name,
			string const & instance_name) const;
    
    /**
       Find an instance by sub-type, without necessarily knowing the
       exact type that it was created for. Particularly useful for
       retrieving base class pointers for things that were registered
       using a more specific sub-class. This method first tries the
       exactly matching type entry, then iterates over all registered
       creators, and then all registered singletons until it finds an
       instance that matches.  For non-singletons, a match implies a
       matching name, for singletons an empty instance_name argument
       serves as a wildcard.
       
       If you know that you are looking for a singleton, it is better
       to use one of the findSingleton() methods.
       
       \note The factory has no notion of type inheritance.  In case
       of ambiguity, you get whichever castable sub-subtype happens to
       be encountered first.
    */
    template<class SubType>
    SubType * find(string const & instance_name) const
    {
      dict_t::const_iterator id(type_code_to_name_.find(typeid(SubType).name()));
      if (type_code_to_name_.end() != id) {
	return dynamic_cast<SubType*> (find (id->second, instance_name));
      }
      for (creator_t::const_iterator ic (creator_.begin()); ic != creator_.end(); ++ic) {
	SubType * instance (dynamic_cast <SubType*> (ic->second->find(instance_name)));
	if (instance) {
	  return instance;
	}
      }
      Configurable * sgl (findSingleton(id->second));
      if (sgl && (instance_name.empty() || instance_name == sgl->name)) {
	return dynamic_cast <SubType*> (sgl);
      }
      return 0;
    }
    
    Configurable * findSingleton(string const & type_name) const;
    
    template<class SubType>
    SubType * findSingleton() const
    {
      dict_t::const_iterator id(type_code_to_name_.find(typeid(SubType).name()));
      if (type_code_to_name_.end() != id) {
        return dynamic_cast<SubType*> (findSingleton (id->second));
      }
      return 0;
    }
    
    /**
       Alternative find method, which uses only the instance name. If
       there are instances of several types which have the same
       instance name, then the first one found is returned.  It first
       tries all the non-singletons, and then the singletons.
       
       \return The existing instance, or zero if the instance name did
       not match.
    */
    Configurable * find(string const & instance_name) const;
    
    template<class SubType>
    Registry<SubType> const * findRegistry() const
    {
      dict_t::const_iterator id(type_code_to_name_.find(typeid(SubType).name()));
      if (type_code_to_name_.end() == id) {
	return 0;
      }
      creator_t::const_iterator ic(creator_.find(id->second));
      if (creator_.end() == ic) {
	return 0;
      }
      Creator<SubType> const * creator (dynamic_cast<Creator<SubType> const *>(ic->second));
      if (0 == creator) {
	return 0;
      }
      return dynamic_cast<Registry<SubType> const *>(&creator->registry_);
    }
    
    /**
       Debug method to write all the registered Configurable subclasses
       to the provided ostream.
    */
    void dump(string const & prefix, ostream & os) const;
    
    
  protected:
    typedef map<string, BaseCreator * > creator_t; // key: type name 
    creator_t creator_;
    typedef map<string, Configurable * > singleton_t; // key: type name
    singleton_t singleton_;
    typedef map<string, string> dict_t;
    dict_t type_code_to_name_; // XXXX to do: do we need a multimap here?
  };
  
}

#endif // FPPLIB_FACTORY_HPP
