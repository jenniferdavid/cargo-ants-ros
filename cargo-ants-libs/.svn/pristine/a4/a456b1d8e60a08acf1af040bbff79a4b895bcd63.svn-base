/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "Argtool.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>


using namespace std;
using namespace boost;


extern "C" {
#include <getopt.h>
}


namespace npm {

Argtool::BoolCallback::
BoolCallback(bool & option,
	     char shortopt,
	     const char * longopt_name,
	     const char * description)
  : BaseCallback(shortopt, longopt_name, false, description),
    _option(option)
{
}


void Argtool::BoolCallback::
operator () (const char * argument)
  throw(runtime_error)
{
  _option = true;
}


Argtool::BaseCallback::
BaseCallback(char shortopt,
	     const char * longopt_name,
	     bool requires_arg,
	     const char * description)
  : _shortopt(shortopt),
    _longopt_name(longopt_name),
    _requires_arg(requires_arg),
    _description(description),
    _longopt(new option())
{
  _longopt->name = _longopt_name.c_str();
  _longopt->has_arg = _requires_arg ? required_argument : no_argument;
  _longopt->flag = NULL;
  _longopt->val = static_cast<int>(_shortopt);
}


Argtool::BaseCallback::
~BaseCallback()
{
  delete _longopt;
}


Argtool::
Argtool()
  : _longest_longopt(0)
{
}


void Argtool::
Add(shared_ptr<BaseCallback> callback)
  throw(runtime_error)
{
  if(_index_map.find(callback->_shortopt) != _index_map.end()){
    ostringstream os;
    os << "Argtool::Add(): Option '" << callback->_shortopt
       << "' already defined";
    throw runtime_error(os.str());
  }

  _index_map.insert(make_pair(callback->_shortopt, _callback.size()));

  if(callback->_longopt_name.size() > _longest_longopt)
    _longest_longopt = callback->_longopt_name.size();  

  _callback.push_back(callback);
  _present.push_back(false);	// will be set in Parse()
  _argument.push_back(0);	// will be set in Parse()
}


int Argtool::
Parse(int argc,
      char ** argv,
      ostream * dbgos)
  throw(runtime_error)
{
  // construct longopt and shortopt
  option longopt[_callback.size() + 1];
  string shortopt;
  for(unsigned int i(0); i < _callback.size(); ++i){
    longopt[i] = * _callback[i]->_longopt;
    shortopt.push_back(_callback[i]->_shortopt);
    if(_callback[i]->_requires_arg)
      shortopt.push_back(':');
  }
  longopt[_callback.size()].name =    NULL;
  longopt[_callback.size()].has_arg = 0;
  longopt[_callback.size()].flag =    NULL;
  longopt[_callback.size()].val =     0;

  // DEBUG
  if(dbgos != 0){
    (*dbgos) << "\nINFO from Argtool::Parse()\n"
	     << "  longopt:\n";
    for(unsigned int i(0); i < _callback.size(); ++i){
      (*dbgos) << "    " << i << ": " << longopt[i].name << " / ";
      if(longopt[i].has_arg == required_argument)
	(*dbgos) << "arg   / ";
      else
	(*dbgos) << "noarg / ";
      (*dbgos) << (char) longopt[i].val << "\n";
    }
    (*dbgos) << "  command line:\n";
    for(int i(0); i < argc; ++i)
      (*dbgos) << "    " << i << ": " << (int *) argv[i]
	       << " / " << argv[i] << "\n";
    (*dbgos) << "  parsing:\n";
  }
  
  // parse command line into callback references and argument pointers
  int res;
  while(1){
    res = getopt_long(argc, argv, shortopt.c_str(), longopt, NULL);

    // DEBUG
    if(dbgos != 0)
      (*dbgos) << " => res:    " << res << " / " << (char) res << "\n";
    
    // check for errors and end of options
    if((res == '?') || (res == ':')){
      ostringstream os;
      os << "Argtool::Parse(): Problems with option '"
	 << (char) optopt << "'";
      throw runtime_error(os.str());
    }
    if(res == -1)
      break;

    // find corresponding callback index
    index_map_t::const_iterator im(_index_map.find(static_cast<char>(res)));
    if(im == _index_map.end())
      throw runtime_error("Argtool::Parse(): Inconsistent index map.");
    unsigned int index(im->second);

    // DEBUG
    if(dbgos != 0)
      (*dbgos) << "    index:  " << index << "\n"
	       << "    optarg: " << (int *) optarg << "\n"
	       << "    optind: " << optind << "\n";
    
    // check for errors
    if(index >= _callback.size())
      throw runtime_error("Argtool::Parse(): Option index mismatch.");
    
    // flag as present
    _present[index] = true;
    if(_callback[index]->_requires_arg)
      _argument[index] = optarg;
  }
  
  // run present callbacks in order of calls to Argtool::Add()
  for(unsigned int i(0); i < _callback.size(); ++i)
    if(_present[i])
      (*_callback[i])(_argument[i]);
  
  return optind;
}


void Argtool::
UsageMessage(ostream & os,
	     const string & synopsis)
{
  os << "\nSynopsis: " << synopsis << "\nOptions:\n";
  
  for(unsigned int i(0); i < _callback.size(); ++i){
    os << "  -" << _callback[i]->_shortopt
       << "  --" << setw(_longest_longopt) << left
       << _callback[i]->_longopt_name;
    if(_callback[i]->_requires_arg)
      os << " <arg>  ";
    else
      os << "        ";
    os << _callback[i]->_description << "\n";
  }
}

}
