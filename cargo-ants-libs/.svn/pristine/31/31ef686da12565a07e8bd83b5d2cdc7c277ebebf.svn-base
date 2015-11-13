/* 
 * Copyright (C) 2005
 * Centre National de Recherche Scientifique, France.
 * All rights reserved.
 * 
 * Developed at
 * Laboratoire d'Automatique et d'Analyse des Systemes, LAAS-CNRS.
 * Visit our homepage at http://www.laas.fr/
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#include "OptionDictionary.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <limits>

using namespace std;

namespace sfl {


  string OptionDictionary::
  GetOption(const string & key) const
  {
    option_t::const_iterator io(m_option.find(key));
    if(io == m_option.end())
      return string("");
    return io->second;
  }


  void OptionDictionary::
  SetOption(const string & key, const string & value)
  {
    option_t::iterator io(m_option.find(key));
    if(io == m_option.end())
      m_option.insert(make_pair(key, value));
    else
      io->second = value;
  }
  
  
  void OptionDictionary::
  ReadFlatText(std::istream & is) throw(std::runtime_error)
  {
    string key;
    while (is >> key){
      if (key[0] == '#') {
	is.ignore(numeric_limits<streamsize>::max(), '\n');
	continue;
      }
      string value;
      while (is >> value) {
	if (key[0] == '#') {
	  is.ignore(numeric_limits<streamsize>::max(), '\n');
	  continue;
	}
	break;
      }
      if ( ! is) {
	ostringstream eos;
	eos << "sfl::OptionDictionary::ReadFlatText(): no value for key \"" << key << "\"";
	throw runtime_error(eos.str());
      }
      SetOption(key, value);
    }
  }
  
  
  void OptionDictionary::
  ReadFlatFile(std::string const & filename) throw(std::runtime_error)
  {
    ifstream fileis(filename.c_str());
    if ( ! fileis) {
      ostringstream eos;
      eos << "sfl::OptionDictionary::ReadFlatFile(): error opening file \"" << filename << "\"";
      throw runtime_error(eos.str());
    }
    ReadFlatText(fileis);
  }
  
  
  void OptionDictionary::
  WriteFlatText(std::string const & prefix, std::ostream & os) throw(std::runtime_error)
  {
    for (option_t::iterator io(m_option.begin()); io != m_option.end(); ++io) {
      if ( ! os) {
	ostringstream eos;
	eos << "sfl::OptionDictionary::WriteFlatText(): output stream not ready (k: \""
	    << io->first << "\" v:\"" << io->second << "\")";
	throw runtime_error(eos.str());
      }
      os << prefix << io->first << "\t" << io->second << "\n";
    }
  }
  
  
  void OptionDictionary::
  WriteFlatFile(std::string const & prefix, std::string const & filename, bool append)
    throw(std::runtime_error)
  {
    ofstream fileos;
    if (append)
      fileos.open(filename.c_str(), ios_base::app);
    else
      fileos.open(filename.c_str());
    if ( ! fileos) {
      ostringstream eos;
      eos << "sfl::OptionDictionary::WriteFlatFile(): error opening file \"" << filename << "\"";
      throw runtime_error(eos.str());
    }
    WriteFlatText(prefix, fileos);
  }
  
}
