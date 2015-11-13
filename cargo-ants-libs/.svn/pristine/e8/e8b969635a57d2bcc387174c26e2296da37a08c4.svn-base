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
//#include <fpplib/yaml_parser.hpp>
#include <iostream>
#include <err.h>
#include <stdlib.h>


class Example
  : public fpplib::Configurable
{
public:
  Example(std::string const & name)
    : fpplib::Configurable(name),
      an_int_(17),
      a_double_(42.0)
  {
    reflectParameter("an_int", &an_int_);
    reflectParameter("a_double", &a_double_);
    std::cout << "constructed an Example called " << name << "\n"
	      << "  an int:   " << an_int_ << "\n"
	      << "  a double: " << a_double_ << "\n";
  }
  
  virtual ~Example()
  {
    std::cout << "Byebye from the Example called " << name << "\n"
	      << "  an int:   " << an_int_ << "\n"
	      << "  a double: " << a_double_ << "\n";
  }
  
private:
  int an_int_;
  double a_double_;
};


class Factory
  : public fpplib::Factory
{
public:
  Factory()
  {
    declare<Example>("example");
  }
};


int main(int argc, char ** argv)
{
  Factory ff;
  fpplib::Configurable * cfg = ff.create("example", "an_example");
  if( ! cfg)
    errx(EXIT_FAILURE, "failed to create example");
  
  std::cout << "Use the top-level Configurable interface to talk to individual parameters:\n";
  fpplib::Parameter<int> * an_int = cfg->lookupParameter<int>("an_int");
  if ( ! an_int)
    errx(EXIT_FAILURE, "failed to lookup appropriately typed int parameter");
  std::cout << "  an_int before: " << *an_int->get() << "\n";
  if ( ! an_int->set(4321))
    errx(EXIT_FAILURE, "failed to set int parameter");
  std::cout << "  an_int after: " << *an_int->get() << "\n";
  
  return 0;
}
