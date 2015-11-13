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

#include <fpplib/yaml_parser.hpp>
#include <fpplib/configurable.hpp>
#include <fstream>


using namespace std;


namespace fpplib {
  
  
  YamlParser::
  YamlParser(Factory & factory)
    : dbg(0),
      factory_(factory)
  {
    addConverter<int>();
    addConverter<long>();
    addConverter<bool>();
    addConverter<size_t>();
    addConverter<ssize_t>();
    addConverter<float>();
    addConverter<double>();
    addConverter<string>();
  }
  
  
  void YamlParser::
  addConverter(BaseValueConverter * bvc)
  {
    converters_.add(bvc->type, bvc);
  }
  
  
  bool YamlParser::
  parseString(string const & yaml_string,
	      ostream & erros)
  {
    istringstream is(yaml_string);
    return parseStream(is, erros);
  }
  
  
  bool YamlParser::
  parseFile(string const & yaml_filename,
	    ostream & erros)
  {
    ifstream is(yaml_filename.c_str());
    if ( ! is) {
      erros << "could not open file `" << yaml_filename << "' for reading\n";
      return false;
    }
    return parseStream(is, erros);
  }
  
  
  bool YamlParser::
  processParameter(BaseParameter * pp,
		   YAML::Node const & value,
		   ostream & erros)
  {
    if (dbg) {
      *dbg << "  looking up converter for parameter " << pp->name << " : " << pp->type << "\n";
    }
    
    BaseValueConverter * cc(converters_.find(pp->type));
    if (0 == cc) {
      erros << "no converter for parameter type `" << pp->type
	    << "' of parameter `" << pp->name << "'\n";
      return false;
    }
    
    if (dbg) {
      *dbg << "  parsing...\n";
    }
    
    if ( ! cc->parse(value, pp, erros)) {
      erros << "failed to parse parameter '" << pp->name << "'\n";
      return false;
    }
    
    if (dbg) {
      pp->dump("    ", *dbg);
    }
    
    return true;
  }
  
  
  bool YamlParser::
  processCallback(BaseCallback * cb,
		  YAML::Node const & value,
		  ostream & erros)
  {
    if (dbg) {
      *dbg << "  looking up converter for callback " << cb->name << " : " << cb->type << "\n";
    }
    
    BaseValueConverter * cc(converters_.find(cb->type));
    if (0 == cc) {
      erros << "no converter for callback type `" << cb->type
	    << "' of callback `" << cb->name << "'\n";
      return false;
    }
    
    if (dbg) {
      *dbg << "  parsing...\n";
    }
    
    if ( ! cc->parse(value, cb->argptr_, erros)) {
      erros << "failed argument parse for callback `" << cb->name << "'\n";
      return false;
    }
    
    if (dbg) {
      cb->dump("    ", *dbg);
    }
    
    return cb->call(erros);
  }


  bool YamlParser::
  processSlot(BaseSlot * ss,
	      YAML::Node const & value,
	      ostream & erros)
  {
    if (dbg) {
      *dbg << "  processing slot '" << ss->name << "'\n";
    }
    
    string configurable_name;
    value >> configurable_name;
    
    if (dbg) {
      *dbg << "  looking up configurable: " << configurable_name << "\n";
    }
    
    Configurable * cc(factory_.find(configurable_name));
    
    if (0 == cc) {
      erros << "no instance `" << configurable_name
	    << "' for slot `" << ss->name << "'\n";
      return false;
    }
    
    if (dbg) {
      *dbg << "  trying to assign to slot...\n";
    }
    
    if ( ! ss->set(cc)) {
      erros << "failed to assign instance `" << configurable_name
	    << "' to slot `" << ss->name << "'\n";
      return false;
    }
    
    if (dbg) {
      ss->dump("    ", *dbg);
    }
    
    return true;
  }
  
  
  bool YamlParser::
  configure(Configurable * instance,
	    YAML::Node const & dict,
	    ostream & erros)
  {
    for (YAML::Iterator irefl(dict.begin()); irefl != dict.end(); ++irefl) {
      string reflectable_name;
      irefl.first() >> reflectable_name;
      if ("name" == reflectable_name) {
	continue;	// already handled during instance construction
      }
      
      if (dbg) {
	*dbg << "  looking up reflectable: " << reflectable_name << "\n";
      }
      
      Reflectable * rr(instance->lookup(reflectable_name));
      if (0 == rr) {
	erros << "unknown reflectable `" << reflectable_name
	      << "' in instance `" << instance->name << "'\n";
	return false;
      }
      
      BaseParameter * pp(dynamic_cast<BaseParameter*>(rr));
      if (pp) {
	if ( ! processParameter(pp, irefl.second(), erros)) {
	  return false;
	}
	continue;
      }
      
      BaseCallback * cb(dynamic_cast<BaseCallback*>(rr));
      if (cb) {
	if ( !cb->sequence_mode) {
	  if ( ! processCallback(cb, irefl.second(), erros)) {
	    return false;
	  }
	  continue;
	}
	YAML::NodeType::value const nt(irefl.second().Type());
	if (nt == YAML::NodeType::Scalar) {
	  if ( ! processCallback(cb, irefl.second(), erros)) {
	    return false;
	  }
	  continue;
	}
	if (nt == YAML::NodeType::Sequence) {
	  for (YAML::Iterator ival(irefl.second().begin()); ival != irefl.second().end(); ++ival) {
	    if ( ! processCallback(cb, *ival, erros)) {
	      return false;
	    }
	  }
	  continue;
	}
	erros << "callback `" << cb->name << "' must be scalar or sequence\n";
	return false;
      }
      
      BaseSlot * ss(dynamic_cast<BaseSlot*>(rr));
      if (ss) {
	YAML::NodeType::value const nt(irefl.second().Type());
	if (nt == YAML::NodeType::Scalar) {
	  if ( ! processSlot(ss, irefl.second(), erros)) {
	    return false;
	  }
	  continue;
	}
	if (nt == YAML::NodeType::Sequence) {
	  for (YAML::Iterator ival(irefl.second().begin()); ival != irefl.second().end(); ++ival) {
	    if ( ! processSlot(ss, *ival, erros)) {
	      return false;
	    }
	  }
	  continue;
	}
	erros << "slot `" << ss->name << "' must be scalar or sequence\n";
	return false;
      }
      
      erros << "BUG? unhandled reflectable type `" << rr->type
	    << "' for `" << reflectable_name
	    << "' in instance `" << instance->name << "'\n";
      return false;
    }
    
    return true;
  }
  
  
  bool YamlParser::
  parseStream(istream & yaml_istream,
	      ostream & erros)
  {
    try {
      YAML::Parser parser(yaml_istream);
      YAML::Node doc;
      
      while (parser.GetNextDocument(doc)) {
	
	if (dbg) {
	  *dbg << "got a document...\n";
	}
	
	for (YAML::Iterator itop(doc.begin()); itop != doc.end(); ++itop) {
	  
	  if (dbg) {
	    *dbg << "got a top-level node from the document...\n";
	  }
	  
	  for (YAML::Iterator ientity(itop->begin()); ientity != itop->end(); ++ientity) {
	    
	    string type_name;
	    ientity.first() >> type_name;
	    Configurable * instance(factory_.findSingleton(type_name));
	    
	    if (instance) {
	      if (dbg) {
		*dbg << "* singleton of type: " << type_name << "\n";
	      }
	    }
	    else {
	      string instance_name;
	      ientity.second()["name"] >> instance_name;
	      instance = factory_.create(type_name, instance_name);
	      if ( ! instance) {
		erros << "unknown type `" << type_name
		      << "' for instance `" << instance_name << "'\n";
		return false;
	      }
	      if (dbg) {
		*dbg << "* type: " << type_name << "\n"
		     << "  instance: " << instance_name << "\n";
	      }
	    }
	    
	    if ( ! configure(instance, ientity.second(), erros)) {
	      return false;
	    }
	    
	  } // end for (ientity)
	} // end for (itop)
      } // end while (get next document)
    } // end try
    catch (YAML::Exception const & ee) {
      erros << "YAML::Exception: " << ee.what() << "\n";
      return false;
    }
    catch (runtime_error const & ee) {
      erros << "runtime error: " << ee.what() << "\n";
      return false;
    }
    return true;
  }
  
}
