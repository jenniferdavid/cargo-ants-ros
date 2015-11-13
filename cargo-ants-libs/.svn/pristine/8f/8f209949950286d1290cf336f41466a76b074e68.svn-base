/* 
 * Copyright (C) 2014 Roland Philippsen. All rights resevred.
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

#ifndef NPM_PLUGIN_HPP
#define NPM_PLUGIN_HPP

#include <fpplib/configurable.hpp>
#include <string>
#include <iosfwd>


extern "C" {

  /**
     Plugin initialization function. Must be implemented by
     plugins. We rely on C binding to avoid C++ compiler specific name
     mangling.
  */
  int npm_plugin_init (void);
  
  /**
     Optional plugin finalization function.
  */
  void npm_plugin_fini (void);
  
  typedef int (*npm_plugin_init_t)(void);
  typedef void (*npm_plugin_fini_t)(void);
  
}


namespace npm {
  
  class Plugin
    : public fpplib::Configurable
  {
  public:
    explicit Plugin(std::string const & name);
    ~Plugin();
    
    bool search (std::string const & spec);
    bool load (std::string const & filename, std::ostream & erros, bool terse = false);
    
  private:
    void * dl_;
    npm_plugin_init_t init_;
    npm_plugin_fini_t fini_;
  };
  
}

#endif // NPM_PLUGIN_HPP
