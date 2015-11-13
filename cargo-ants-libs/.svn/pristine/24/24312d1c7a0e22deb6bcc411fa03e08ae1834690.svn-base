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

#ifndef FPPLIB_YAML_PARSER_HPP
#define FPPLIB_YAML_PARSER_HPP

#include <fpplib/factory.hpp>
#include <fpplib/parameter.hpp>
#include <fpplib/callback.hpp>
#include <fpplib/slot.hpp>
#include <yaml-cpp/yaml.h>
#include <iosfwd>


namespace fpplib {

  using std::string;
  using std::istream;
  
  
  /**
     Base class for ValueConverter, which is a helper construct to
     parse values from the YAML file into parameters and slots.
  */
  class BaseValueConverter
  {
  public:
    string const type;
    
    explicit BaseValueConverter(string const & type_)
      : type(type_)
    {
    }
    
    virtual ~BaseValueConverter()
    {
    }
    
    virtual bool parse(YAML::Node const & node,
		       BaseParameter * parameter,
		       ostream & erros) const = 0;
  };
  
  
  /**
     Templatized value converter for value types that can be read
     using the standard input operators.
  */
  template<typename value_type>
  class ValueConverter
    : public BaseValueConverter
  {
  public:
    ValueConverter()
      : BaseValueConverter(typeid(value_type).name())
    {
    }
    
    /**
       \todo There should be a way to say more than just
       success/failure here.  Particularly when using parameter guards
       to fend off i.e. negative values.
    */
    virtual bool parse(YAML::Node const & node, BaseParameter * parameter, ostream & erros) const
    {
      Parameter<value_type> * pp(dynamic_cast<Parameter<value_type> * >(parameter));
      if (0 == pp) {
	erros << "type mismatch: expected " << type << " but got " << parameter->type;
	return false;
      }
      value_type value;
      node >> value;
      return pp->set(value, erros);
    }
  };
  
  
  /**
     Templatized value converter for vectors of value types that can
     be read using the standard input operators.
  */
  template<typename value_type>
  class VectorConverter
    : public BaseValueConverter
  {
  public:
    typedef typename std::vector<value_type> vector_type;
    
    VectorConverter()
      : BaseValueConverter(typeid(vector_type).name())
    {
    }
    
    /**
       \todo XXXX to do: if we add guards for vector parameters, this
       method will get significantly more involved because we should
       then not give blanket access to the underlying vector instance.
    */
    virtual bool parse(YAML::Node const & node, BaseParameter * parameter, ostream & erros) const
    {
      VectorParameter<value_type> * pp(dynamic_cast<VectorParameter<value_type> * >(parameter));
      if (0 == pp) {
	erros << "type mismatch: expected " << type << " but got " << parameter->type;
	return false;
      }
      vector_type * vv(pp->get());
      if (0 == vv) {
	erros << "no vector instance";
	return false;
      }
      vv->clear();
      for (YAML::Iterator ival(node.begin()); ival != node.end(); ++ival) {
	value_type value;
	(*ival) >> value;
	vv->push_back(value);
      }
      return true;
    }
    
  };
  
  
  /**
     Parser for YAML files. You have to register value converters for
     each type that you expect to encounter. The YamlParser comes
     built in with some commonly encountered types, such as int,
     double, size_t, and string.
     
     \todo XXXX to do: automate the value converter generation process
     as much as possible. For instance, it should be rather
     straightforward to come up with a template that converts YAML
     lists to std::vector<>, or with a way to assign struct fields as
     name-value combinations from YAML maps.
  */
  class YamlParser
  {
  public:
    YamlParser(Factory & factory);
    
    /** \note transfers ownership of \param bvc */
    void addConverter(BaseValueConverter * bvc);
    
    /**
       Convenience method for adding custom type converters. Takes
       care of vector converters, too.
    */
    template<typename value_type>
    void addConverter() {
      addConverter(new ValueConverter<value_type>());
      addConverter(new VectorConverter<value_type>());
    }
    
    bool parseString(string const & yaml_string, ostream & erros);
    bool parseFile(string const & yaml_filename, ostream & erros);
    bool parseStream(istream & yaml_istream, ostream & erros);
    
    ostream * dbg;
    
  protected:
    Registry<BaseValueConverter> converters_;
    Factory & factory_;
    
    bool configure(Configurable * instance, YAML::Node const & dict, ostream & erros);
    bool processParameter(BaseParameter * pp, YAML::Node const & value, ostream & erros);
    bool processCallback(BaseCallback * cb, YAML::Node const & value, ostream & erros);
    bool processSlot(BaseSlot * ss, YAML::Node const & value, ostream & erros);
  };
  
}

#endif // FPPLIB_YAML_PARSER_HPP
