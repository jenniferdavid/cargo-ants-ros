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

#include <fpplib/configurable.hpp>
#include <fpplib/factory.hpp>
#include <fpplib/yaml_parser.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <err.h>

using namespace fpplib;
using namespace std;


namespace {
  
  
  class TestCB
    : public Configurable
  {
  public:
    TestCB (string const & name)
      : Configurable (name)
    {
      reflectCallback<double> ("some_double", true, boost::bind(&TestCB::someDouble, this, _1));
      reflectCallback<int> ("some_int", false, boost::bind(&TestCB::someIntRef, this, _1));
    }
    
    bool someDouble(double some_double)
    {
      cout << "someDouble in " << name << ": " << some_double << "\n";
      return true;
    }
    
    bool someIntRef(int const some_int)
    {
      cout << "someIntRef in " << name << ": " << some_int << "\n";
      return true;
    }
  };
  
  
  class TestCBFactory
    : public Factory
  {
  public:
    TestCBFactory()
    {
      declare<TestCB>("testcb");
    }
  };
  
}


int main (int argc, char **argv)
{
  TestCBFactory ff;
  YamlParser pp(ff);
  pp.dbg = &cout;
  if ( ! pp.parseString("[ { testcb: { name: hello, some_double: 9.87, some_int: 42 } }, { testcb: { name: byebye, some_double: [ -9, -10, -11 ] } } ]")) {
    cout << "parse error: " << pp.error << "\n";
    return 42;
  }
  
  ff.dump("  ", cout);
  return 0;
}
