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

#ifndef FPPLIB_CONFIGURABLE_HPP
#define FPPLIB_CONFIGURABLE_HPP

#include <fpplib/registry.hpp>
#include <fpplib/parameter.hpp>
#include <fpplib/callback.hpp>
#include <fpplib/slot.hpp>
#include <string>
#include <iosfwd>


namespace fpplib {
  

  /**
     Base class for configurable instances. Within ffplib,
     configurable means that for a given Configurable instance, its
     Reflectable attributes can be looked up by name. Reflectable
     attributes can be Parameter<> instances that represent values,
     Callback<> instances that allow the Configurable class to be
     notified in a custom manner when a parameter is set, an Slot<>
     instances that have builtin polymorphism support (you can fill a
     slot with an instance of a subclass of the declared type).
     
     Client code will subclass Configurable, and call
     Configurable::reflectParameter,
     Configurable::reflectVectorParameter,
     Configurable::reflectCallback, Configurable::reflectSlot, or
     Configurable::reflectVectorSlot in order to make some of their
     attributes reachable from the outside. Note that further variants
     of the reflect methods may have been added since this
     documentation was written.
  */
  class Configurable
  {
  public:
    /**
       Every Confiburable instance has a name. This is useful for
       maintaining registries of instances so that things can be found
       by name.
     */
    string const name;
    
    explicit Configurable(string const & name);
    
    virtual ~Configurable();
    
    /**
       Retrieve a Reflectable attribute from this Configurable
       instance. Any attribute that has been declared by this specific
       Confiburable subtype or any of its ancestors can be retrieved
       in this manner.
       
       \return The Reflectable which matches the given name, or zero
       if there is nothing that goes by that name.
     */
    Reflectable * lookup(string const & name);
    
    /**
       Read-only access to Reflectable attributes. Any attribute that
       has been declared by this specific Confiburable subtype or any
       of its ancestors can be retrieved in this manner.
       
       \return The Reflectable which matches the given name, or zero
       if there is nothing that goes by that name.
    */
    Reflectable const * lookup(string const & name) const;
    
    /**
       Convenience method to look up a Parameter and attempt to cast
       it to a given type. When this method returns zero, either no
       parameter with the given name is reflected, or that particular
       parameter cannot be cast to the desired type.
    */
    template<typename value_type>
    Parameter<value_type> * lookupParameter(string const & name)
    {
      return dynamic_cast<Parameter<value_type>*>(lookup(name));
    }
    
    /**
       Convenience method to look up a const Parameter and attempt to
       cast it to a given type. When this method returns zero, either
       no parameter with the given name is reflected, or that
       particular parameter cannot be cast to the desired type.
    */
    template<typename value_type>
    Parameter<value_type> * const lookupParameter(string const & name) const
    {
      return dynamic_cast<Parameter<value_type>*>(lookup(name));
    }
    
    /**
       Debug method to write all reflected attributes to the provided
       ostream.
    */
    virtual void dump(string const & prefix,
		      ostream & os) const;
    
  private:
    /**
       Registry of everything that is reflected by this Configurable.
       
       \note This field used to be protected instead of private.
       Change it back in case that causes more headaches than worth
       it.
    */
    Registry<Reflectable> reflected_;
    
  protected:
    /**
       Subclasses use this method to declare a parameter that should
       be reflected.  You declare your parameters by giving them a
       name, providing a pointer to the field where you store them,
       and optionally a pointer to a ParameterGuard which will be
       consulted before values are written to the reflected parameter.
       
       \note There is a collection of similar methods, depending on
       what you are trying to reflect. At the time of writing, there
       also is Configurable::reflectVectorParameter,
       Configurable::reflectCallback, Configurable::reflectSlot, and
       Configurable::reflectVectorSlot.
       
       \return A freshly allocated Parameter<> instance that
       represents the given parameter instance.
    */
    template<typename value_type>
    Parameter<value_type> * reflectParameter(string const & name,
					     value_type * instance,
					     ParameterGuard * guard = 0)
    {
      Parameter<value_type> * pp(new Parameter<value_type>(name, instance, guard));
      reflected_.add(name, pp);
      return pp;
    }
    
    /**
       Use this method in your subclasses to declare parameters that
       contain vectors of identically typed values. For
       polymorphically typed values, see reflectVectorSlot.  You
       declare your parameters by giving them a name and providing a
       pointer to the std::vector<> where you store them.
       
       \note There is a collection of similar methods, depending on
       what you are trying to reflect. At the time of writing, there
       also is Configurable::reflectParameter,
       Configurable::reflectCallback, Configurable::reflectSlot, and
       Configurable::reflectVectorSlot.
       
       \todo Consider adding support for ParameterGuard here.
       
       \return A freshly allocated Parameter<> instance that
       represents the given parameter instance.
    */
    template<typename value_type>
    VectorParameter<value_type> * reflectVectorParameter(string const & name,
							 std::vector<value_type> * instance)
    {
      VectorParameter<value_type> * pp(new VectorParameter<value_type>(name, instance));
      reflected_.add(name, pp);
      return pp;
    }
    
    /**
       If the value- or slot- based reflection mechanism is not
       flexible enough for your needs, you can use a callback
       instead. Callbacks allow you to declare something that appears
       like a parameter in e.g. configuration files, but when the
       parameter is written to your callback ends up being
       called. Typical usage is to give a functor as callable_type,
       for example using boost::bind.
       
       \code
       class Example
         : public Configurable
       {
       public:
         explicit Example (string const & name)
	   : Configurable (name)
	 {
	   reflectCallback<string> ("foo", false, boost::bind(&Example::foo, this, _1));
	   reflectCallback<int> ("bar", true, boost::bind(&Example::bar, this, _1));
	 }
	 
	 bool foo (string const & the_value)
	 { cout << "callback foo with value " << the_value << "\n"; }
	 
	 bool bar (int the_value)
	 { cout << "callback bar with value " << the_value << "\n"; }
       };
       \endcode
       
       \note There is a collection of similar methods, depending on
       what you are trying to reflect. At the time of writing, there
       also is Configurable::reflectParameter,
       Configurable::reflectVectorParameter,
       Configurable::reflectSlot, and Configurable::reflectVectorSlot.
       
       \todo The sequence_mode parameter is a bit of a hack in order
       to not skip values when parsing YAML files, but it does point
       to a deeper question: when several name-value pairs have the
       same name, do you want to be notified of each of them, or only
       one of them (e.g. the first encountered or the last
       encountered)? If you want all of them, set sequence_mode to
       true. When in doubt, you probably want to set it true, and
       figure out how to handle multiple assignments inside of your
       callback implementation.
       
       \return A freshly allocated Parameter<> instance that
       represents the given parameter instance.
    */
    template<typename value_type, typename callable_type>
    Callback<value_type, callable_type> * reflectCallback(string const & name,
							  bool sequence_mode,
							  callable_type callback)
    {
      Callback<value_type, callable_type> * cb;
      cb = new Callback<value_type, callable_type>(name, sequence_mode, callback);
      reflected_.add(name, cb);
      return cb;
    }
    
    /**
       Utility method for subclasses to declare their slots
       (Reflectable objects that can be replaced by subclasses of what
       is required). You declare your slots by giving them a name and
       providing a pointer to the field where you store them (as a
       pointer).
       
       \note There is a collection of similar methods, depending on
       what you are trying to reflect. At the time of writing, there
       also is Configurable::reflectParameter,
       Configurable::reflectVectorParameter,
       Configurable::reflectCallback, and
       Configurable::reflectVectorSlot.
       
       \return A freshly allocated Slot<> instance that represents the
       given slot.
    */
    template<class Type>
    Slot<Type> * reflectSlot(string const & name,
			     Type ** instance)
    {
      Slot<Type> * pp(new Slot<Type>(name, instance));
      reflected_.add(name, pp);
      return pp;
    }
    
    /**
       Utility method to declare a slot which is a std::vector of
       pointers to a given field. Otherwise identical to
       Configurable::declareSlot.
       
       \note There is a collection of similar methods, depending on
       what you are trying to reflect. At the time of writing, there
       also is Configurable::reflectParameter,
       Configurable::reflectVectorParameter,
       Configurable::reflectCallback, and Configurable::reflectSlot.
       
       \return A freshly allocated VectorSlot<> instance that
       represents the given slot.
    */
    template<class Type>
    VectorSlot<Type> * reflectVectorSlot(string const & name,
					 vector<Type *> * instvec)
    {
      VectorSlot<Type> * pp(new VectorSlot<Type>(name, instvec));
      reflected_.add(name, pp);
      return pp;
    }
  };
  
}

#endif // FPPLIB_CONFIGURABLE_HPP
