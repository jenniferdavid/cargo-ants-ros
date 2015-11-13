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


#ifndef NPM_ARGTOOL_HPP
#define NPM_ARGTOOL_HPP


#include <boost/shared_ptr.hpp>
#include <iosfwd>
#include <stdexcept>
#include <vector>
#include <map>
#include <string>
#include <sstream>


struct option;

namespace npm {


/**
   \todo The exception mechanism should be replaced by a return value
   of BaseCallback::operator() and an error message stream.
*/
class Argtool
{
public:
  class BaseCallback {
  public:
    BaseCallback(char shortopt,
		 const char * longopt_name,
		 bool requires_arg,
		 const char * description);
    virtual ~BaseCallback();
    
    /** Overwritten by subclasses to implement the option. */
    virtual void operator () (const char * argument)
      throw(std::runtime_error) = 0;

  protected:
    friend class Argtool;

    const char        _shortopt;
    const std::string _longopt_name;
    const bool        _requires_arg;
    const std::string _description;
    option *          _longopt;
  };

  template<typename T>
  class Callback: public BaseCallback {
  public:
    typedef T option_t;

    inline Callback(option_t & option,
		    char shortopt,
		    const char * longopt_name,
		    const char * description);
    
    inline void operator () (const char * argument) throw(std::runtime_error);
    
  private:
    option_t & _option;
  };

  class BoolCallback: public BaseCallback {
  public:
    BoolCallback(bool & option,
		 char shortopt,
		 const char * longopt_name,
		 const char * description);
    void operator () (const char * argument) throw(std::runtime_error);
    
  private:
    bool & _option;
  };

  
  Argtool();
  
  void Add(boost::shared_ptr<BaseCallback> callback) throw(std::runtime_error);
  
  void Add(BaseCallback * callback) throw(std::runtime_error)
  { Add(boost::shared_ptr<BaseCallback>(callback)); }

  /**
     \return Index into argv of first non-option argument.
     \note Use e.g. dbgos = & cerr to switch on debug messages.
  */
  int Parse(int argc, char ** argv, std::ostream * dbgos)
    throw(std::runtime_error);
  
  void UsageMessage(std::ostream & os, const std::string & synopsis);


private:
  typedef std::map<char, unsigned int> index_map_t;

  index_map_t                  _index_map;  
  std::vector<boost::shared_ptr<BaseCallback> >  _callback;
  std::vector<bool>            _present;
  std::vector<const char *>    _argument;
  unsigned int                 _longest_longopt;
};


template<typename T>
Argtool::Callback<T>::
Callback(option_t & option,
	 char shortopt,
	 const char * longopt_name,
	 const char * description)
  : BaseCallback(shortopt, longopt_name, true, description),
    _option(option)
{
}


template<typename T>
void Argtool::Callback<T>::
operator () (const char * argument)
  throw(std::runtime_error)
{
  if(argument == 0)
    throw std::runtime_error(_longopt_name + ": argument expected");
  std::istringstream is(argument);
  is >> _option;
  if( ! is)
    throw std::runtime_error(_longopt_name
			     + ": invalid argument \"" + argument + "\"");
}

}

#endif // NPM_ARGTOOL_HPP
