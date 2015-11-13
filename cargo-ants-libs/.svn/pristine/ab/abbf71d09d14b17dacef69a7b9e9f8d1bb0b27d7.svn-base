/* 
 * Copyright (C) 2012 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#ifndef SFL2_NPM_FACTORY_HPP
#define SFL2_NPM_FACTORY_HPP

#include <fpplib/factory.hpp>
#include <fpplib/yaml_parser.hpp>


namespace npm {

  class World;
  
  
  class Factory
    : public fpplib::Factory
  {
  private:
    Factory();
    
  public:
    static Factory & Instance();
    
    World * GetWorld();
    fpplib::YamlParser & GetParser();
    
    bool ParseFile(std::string const &yaml_filename,
		   std::ostream &erros,
		   std::ostream *dbgos = 0);

  private:
    fpplib::YamlParser parser_;
  };

}

#endif // SFL2_NPM_FACTORY_HPP
