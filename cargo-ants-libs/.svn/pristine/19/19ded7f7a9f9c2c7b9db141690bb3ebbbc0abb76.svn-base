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

#include <typeinfo>
#include <string>
#include <map>
#include <iostream>


namespace idea {
  
  using std::type_info;
  using std::string;
  using std::map;
  using std::cout;
  
  
  class Reflectable;
  
  struct BaseSlot {
    string const type;
    string const name;
    
    virtual ~BaseSlot() {}
    
    BaseSlot(string const & type_, string const & name_)
      : type(type_), name(name_)
    {}
    
    virtual bool set(Reflectable * instance) = 0;
    virtual void dump() const = 0;
  };
  
  
  template<class Type>
  struct Slot
    : public BaseSlot
  {
    Type ** slot;
    
    Slot(string const & name, Type ** slot_)
      : BaseSlot(typeid(Type).name(), name),
	slot(slot_)
    {}
    
    virtual bool set(Reflectable * instance) {
      if (0 == this) {
	return false;
      }
      Type * bb(dynamic_cast<Type * >(instance));
      if ( ! bb) {
	return false;
      }
      *slot = bb;
      return true;
    }
    
    virtual void dump() const {
      cout << "  " << name << " : " << type << " = " << *slot << "\n";
    }
  };
  
  
  struct Reflectable {
    string const name;
    map<string, BaseSlot * > slots_;
    
    virtual ~Reflectable() {
      for (map<string, BaseSlot * >::iterator ii(slots_.begin()); ii != slots_.end(); ++ii) {
	delete ii->second;
      }
    }
    
    explicit Reflectable(string const & name_)
      : name(name_) {}
    
    template<class Type>
    Slot<Type> * reflect(string const & name,
			 Type ** instance)
    {
      Slot<Type> * slot(new Slot<Type>(name, instance));
      slots_[name] = slot;
      return slot;
    }
    
    BaseSlot * lookup(string const & name)
    {
      map<string, BaseSlot * >::const_iterator ii(slots_.find(name));
      if (slots_.end() == ii) {
	return 0;
      }
      return ii->second;
    }
    
    virtual void dump() {
      cout << name << "\n";
      for (map<string, BaseSlot * >::iterator ii(slots_.begin()); ii != slots_.end(); ++ii) {
	ii->second->dump();
      }
    }
  };
  
  
  class Root : public Reflectable
  {
  public:
    Root(string const & name) : Reflectable(name) {}
    
    virtual void hello() {
      cout << "  hello from root `" + name + "'\n";
    }
  };
  
  
  class Base : public Root
  {
  public:
    Base(string const & name) : Root(name) {}
    
    virtual void hello() {
      cout << "  hello from base `" + name + "'\n";
    }
  };
  
  
  class Sub1 : public Base
  {
  public:
    Sub1(string const & name) : Base(name) {}
    
    virtual void hello() {
      cout << "  hello from sub1 `" + name + "'\n";
    }
  };
  
  
  class Sub2 : public Base
  {
  public:
    Sub2(string const & name) : Base(name) {}
    
    virtual void hello() {
      cout << "  hello from sub2 `" + name + "'\n";
    }
  };
  
  class Slotted : public Reflectable
  {
  public:
    Root * one;
    Base * two;
    Sub1 * three;
    
    Slotted(string const & name) : Reflectable(name), one(0), two(0), three(0)
    {
      reflect("one", &one);
      reflect("two", &two);
      reflect("three", &three);
    }
    
    virtual void hello()
    {
      cout << "hello from slottable `" + name + "'\n";
      cout << " one:\n";
      if (one) {
	one->hello();
      }
      else {
	cout << "  NULL\n";
      }
      cout << " two:\n";
      if (two) {
	two->hello();
      }
      else {
	cout << "  NULL\n";
      }
      cout << " three:\n";
      if (three) {
	three->hello();
      }
      else {
	cout << "  NULL\n";
      }
    }
  };
  
}


using namespace idea;

int main(int argc, char ** argv)
{
  Root root("root");
  Base base("base");
  Sub1 sub1("sub1");
  Sub2 sub2("sub2");
  Slotted slotted("slotted");
  
  cout << "\nfresh off the press\n";
  slotted.hello();
  
  slotted.lookup("one")->set(&root);
  slotted.lookup("two")->set(&root);
  slotted.lookup("three")->set(&root);
  
  cout << "\nafter broadcasting root\n";
  slotted.hello();
  
  slotted.lookup("one")->set(&base);
  slotted.lookup("two")->set(&base);
  slotted.lookup("three")->set(&base);
  
  cout << "\nafter broadcasting base\n";
  slotted.hello();
  
  slotted.lookup("one")->set(&sub1);
  slotted.lookup("two")->set(&sub1);
  slotted.lookup("three")->set(&sub1);
  
  cout << "\nafter broadcasting sub1\n";
  slotted.hello();
  
  slotted.lookup("one")->set(&sub2);
  slotted.lookup("two")->set(&sub2);
  slotted.lookup("three")->set(&sub2);
  
  cout << "\nafter broadcasting sub2\n";
  slotted.hello();
}
