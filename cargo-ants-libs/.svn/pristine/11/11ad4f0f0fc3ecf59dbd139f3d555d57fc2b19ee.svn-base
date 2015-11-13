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

#include <fpplib/configurable.hpp>
#include <fpplib/factory.hpp>
#include <fpplib/yaml_parser.hpp>
#include <iostream>

using namespace fpplib;
using namespace std;


namespace {
  
  struct Base : public Configurable {
    Base(string const & name)
      : Configurable(name), blah_(17)
    {
      reflectParameter("blah", &blah_);
    }
    
    int blah_;
  };
  
  struct Sub1 : public Base {
    Sub1(string const & name)
      : Base(name),
	foo_(-12.32)
    {
      reflectParameter("foo", &foo_);
    }
    
    double foo_;
  };
  
  struct Sub2 : public Base {
    Sub2(string const & name)
      : Base(name),
	bar_("dunno")
    {
      reflectParameter("bar", &bar_);
    }
    
    string bar_;
  };
  
  struct Slotted : public Configurable {
    Base * one;
    Sub1 * two;
    vector<Base *> three;
    
    Slotted(string const & name)
      : Configurable(name), one(0), two(0)
    {
      reflectSlot("one", &one);
      reflectSlot("two", &two);
      reflectVectorSlot("three", &three);
    }
  };
  
  
  class MyFactory
    : public Factory
  {
  public:
    MyFactory()
    {
      declare<Base>("base");
      declare<Sub1>("sub1");
      declare<Sub2>("sub2");
      declare<Slotted>("slotted");
    }
  };
  
}

int main(int argc, char ** argv)
{
  {
    MyFactory ff;
    ff.create("base", "ground");
    ff.create("sub1", "hello");
    ff.create("sub1", "world");
    ff.create("sub2", "byebye");
    ff.create("slotted", "arthur");
    cout << "\nfresh off the press\n";
    ff.dump("  ", cout);
    Slotted * aa(dynamic_cast<Slotted*>(ff.find("slotted", "arthur")));
    if (0 == aa) {
      cout << "Hey, what's wrong with arthur?\n";
    }
    else {
      dynamic_cast<BaseSlot*>(aa->lookup("one"))->set(ff.find("sub2", "byebye"));
      dynamic_cast<BaseSlot*>(aa->lookup("two"))->set(ff.find("sub2", "byebye"));
      cout << "\nafter slotting with sub2/byebye\n";
      ff.dump("  ", cout);
      dynamic_cast<BaseSlot*>(aa->lookup("one"))->set(ff.find("sub1", "hello"));
      dynamic_cast<BaseSlot*>(aa->lookup("two"))->set(ff.find("sub1", "hello"));
      cout << "\nafter slotting with sub1/hello\n";
      ff.dump("  ", cout);
    }
  }
  
  {
    MyFactory ff;
    YamlParser pp(ff);
    pp.dbg = &cout;
    cout << "\nTrying to parse a simple document without any slots\n";
    if ( ! pp.parseString("[ { sub1: { name: hello, foo: 9.87 } }, { sub2: { name: byebye, bar: overtherainbow } } ]")) {
      cout << "parse error: " << pp.error << "\n";
    }
    else {
      ff.dump("  ", cout);
    }
  }
  
  {
    MyFactory ff;
    YamlParser pp(ff);
    pp.dbg = &cout;
    cout << "\nTrying to parse a document with slots\n";
    if ( ! pp.parseString("- sub1:\n"
			  "    name: hello\n"
			  "    foo: 9.87\n"
			  "- sub2:\n"
			  "    name: byebye\n"
			  "    bar: overtherainbow\n"
			  "- slotted:\n"
			  "    name: brian\n"
			  "    one: byebye\n"
			  "    two: hello\n"
			  "    three: [ byebye, hello, hello, byebye ]")) {
      cout << "parse error: " << pp.error << "\n";
    }
    else {
      ff.dump("  ", cout);
    }
  }
}
