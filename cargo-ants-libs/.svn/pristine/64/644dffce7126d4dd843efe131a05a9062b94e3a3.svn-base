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

#include "Plugin.hpp"
#include <sfl/util/strutil.hpp>
#include <iostream>
#include <sstream>
#include <list>
#include <boost/bind.hpp>
#include <dlfcn.h>


namespace npm {
  
  
  Plugin::
  Plugin (std::string const & name)
    : fpplib::Configurable (name),
      dl_(0),
      init_(0),
      fini_(0)
  {
    reflectCallback<std::string> ("spec", false, boost::bind (&Plugin::search, this, _1));
  }
  
  
  Plugin::
  ~Plugin()
  {
    if (fini_) {
      fini_();
    }
    if (dl_) {
      dlclose (dl_);
    }
  }
  
  
  bool Plugin::
  load (std::string const & filename, std::ostream & erros, bool terse)
  {
    if (dl_) {
      if (terse) {
	erros << "plugin " << name << " already loaded\n";
      }
      else {
	erros << "npm::Plugin::load cannot load more than one file\n";
      }
      return false;
    }
    
    dl_ = dlopen (filename.c_str(), RTLD_NOW);
    if ( ! dl_) {
      if (terse) {
	erros << filename << ": " << dlerror() << "\n";
      }
      else {
	erros << "npm::Plugin::load cannot dlopen " << filename
	      << ": " << dlerror() << "\n";
      }
      return false;
    }
    
    dlerror();			// make sure dlerror is cleared
    init_ = (npm_plugin_init_t) dlsym (dl_, "npm_plugin_init");
    char const * dlsym_error (dlerror());
    if (dlsym_error) {
      if (terse) {
	erros << filename << ": npm_plugin_init: " << dlerror() << "\n";
      }
      else {
	erros << "npm::Plugin::load invalid npm_plugin_init symbol in " << filename
	      << ": " << dlsym_error << "\n";
      }
      dlclose (dl_);
      dl_ = 0;
      return false;
    }
    
    int const init_error (init_());
    if (0 != init_error) {
      if (terse) {
	erros << filename << ": npm_plugin_init error code " << init_error << "\n";
      }
      else {
	erros << "npm::Plugin::load npm_plugin_init of " << filename
	      << " returned error code " << init_error << "\n";
      }
      dlclose (dl_);
      dl_ = 0;
      return false;
    }
    
    fini_ = (npm_plugin_fini_t) dlsym (dl_, "npm_plugin_fini"); // can be NULL, no need to check
    
    return true;
  }
  
  
  bool Plugin::
  search (std::string const & spec)
  {
    if (spec.empty()) {
      std::cerr << "npm::Plugin::search(): empty spec\n";
      return false;
    }
    
    if (spec[0] == '/') {
      return load (spec, std::cerr);
    }
    
    std::string searchpath;
    
#ifdef NPM_PLUGIN_PATH_STR
    if (getenv("NPM_PLUGIN_PATH")) {
      searchpath = std::string (NPM_PLUGIN_PATH_STR) + ":" + getenv("NPM_PLUGIN_PATH");
    }
    else {
      searchpath = std::string (NPM_PLUGIN_PATH_STR);
    }
#else // NPM_PLUGIN_PATH_STR
    if (getenv("NPM_PLUGIN_PATH")) {
      searchpath = std::string (NPM_PLUGIN_PATH_STR) + ":" + getenv("NPM_PLUGIN_PATH");
    }
#endif // NPM_PLUGIN_PATH_STR
    
    typedef std::list <std::string> directories_t;
    directories_t directories;
    sfl::tokenize (searchpath, ':', directories);
    std::string filename;
    std::ostringstream err;
    
    err << "npm::Plugin::search (`" <<  spec << "`) failed:\n";
    for (directories_t::const_iterator ip (directories.begin()); ip != directories.end(); ++ip) {
      if ( ! ip->empty()) {
	filename = *ip + "/lib" + spec + ".so";
	err << "  ";
	if (load (filename, err, true)) {
	  return true;
	}
	filename = *ip + "/" + spec + ".so";
	err << "  ";
	if (load (filename, err, true)) {
	  return true;
	}
	filename = *ip + "/" + spec;
	err << "  ";
	if (load (filename, err, true)) {
	  return true;
	}
      }
    }
    
    std::cerr << err.str() << "\n";
    return false;
  }
  
}
